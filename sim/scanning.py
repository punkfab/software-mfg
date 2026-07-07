"""scanning.py — 3D scanning as the reality-capture leg: point cloud -> CAD -> pose + deviation.

Scanning is where real geometry re-enters the loop. A depth cam / line laser produces a
point cloud of a real part; registering it to the part's CAD gives two things this repo
already wants a home for:

  1. **6-DoF pose** — the real observation behind `tracking.observe()` / `Floor.fix()`
     (which are fed by simulated ground truth today). Scan -> ICP -> pose re-anchors a part.
  2. **As-built deviation** — per-point signed distance to the CAD surface: the geometry
     version of the calibration writeback (a measured residual vs the nominal), and the
     as-built check behind `assembly.verify`.

Self-contained (trimesh ICP + sampling + proximity, no open3d): a scan is *simulated* from
the CAD ground truth (sample the surface, cull to one view, add sensor noise, place it at
the part's real pose, optionally inject a defect) — the same "observations synthesized from
ground truth now, a real sensor later" discipline as `tracking`. Meshes are mm; poses out
are metres + wxyz quat to match `tracking.world.Pose`. ICP is LOCAL — it needs a coarse
initial pose (nominal / centroid), like any real scan-to-CAD pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh


@dataclass
class Scan:
    points: np.ndarray            # (N,3) mm, in the sensor/world frame
    n_view: int = 1
    noise_mm: float = 0.0


# A few views that cover a boxy part's faces -> full 6-DoF observability (turntable / the
# arm sweeping around the part). A SINGLE flat view is degenerate: a lone plane doesn't
# constrain pose, so ICP can settle a wrong global placement with a deceptively low RMSE.
VIEWS_MULTI = ((0, 0, -1), (0, 0, 1), (-1, 0, 0), (1, 0, 0), (0, -1, 0), (0, 1, 0))
VIEWS_TOP = ((0, 0, -1),)


def simulate_scan(mesh, n_points=4000, pose_mm=None, noise_mm=0.05, view_dirs=VIEWS_MULTI,
                  defect=None, seed=0):
    """Synthesize a scan of `mesh` (mm). Samples the surface, keeps points each sensor view
    can see (front faces per view_dir), adds noise, and places the cloud at the part's REAL
    pose (pose_mm, 4x4). view_dirs=None -> full surface; VIEWS_TOP -> a degenerate single
    view. `defect`=(center_mm, radius_mm, bump_mm) pushes a region proud (as-built warp)."""
    if view_dirs is None:
        pts, face_idx = trimesh.sample.sample_surface(mesh, n_points, seed=seed)
    else:
        per = max(1, n_points // len(view_dirs))
        chunks, faces = [], []
        for i, vd in enumerate(view_dirs):
            p, f = trimesh.sample.sample_surface(mesh, per * 3, seed=seed + i)
            keep = mesh.face_normals[f] @ np.asarray(vd, float) < -0.1
            chunks.append(p[keep][:per]); faces.append(f[keep][:per])
        pts, face_idx = np.vstack(chunks), np.concatenate(faces)
    normals = mesh.face_normals[face_idx]

    if defect is not None:
        c, r, bump = np.asarray(defect[0], float), defect[1], defect[2]
        hit = np.linalg.norm(pts - c, axis=1) < r
        pts = pts.copy(); pts[hit] += normals[hit] * bump    # push the region proud

    if noise_mm:                                     # deterministic pseudo-noise (no RNG global)
        k = np.arange(len(pts))
        jitter = np.stack([np.sin(k * 12.9898 + seed), np.sin(k * 78.233 + seed),
                           np.sin(k * 37.719 + seed)], 1)
        pts = pts + (jitter % 1.0 - 0.5) * 2 * noise_mm

    if pose_mm is not None:
        pts = trimesh.transform_points(pts, np.asarray(pose_mm, float))
    return Scan(points=pts, n_view=1 if view_dirs is None else len(view_dirs), noise_mm=noise_mm)


def register_to_cad(scan, cad_mesh, initial=None, max_iterations=80):
    """ICP-align a scan (mm) to the CAD. Returns {transform, part_pose, rmse_mm, converged}.
    `transform` maps scan->cad; `part_pose` is its inverse = the part's pose vs CAD nominal.
    Seeds ICP with a centroid pre-alignment (coarse init) unless `initial` is given."""
    pts = scan.points if isinstance(scan, Scan) else np.asarray(scan, float)
    if initial is None:
        initial = np.eye(4)
        initial[:3, 3] = cad_mesh.centroid - pts.mean(axis=0)   # coarse translation
    T, aligned, _ = trimesh.registration.icp(pts, cad_mesh, initial=initial,
                                             max_iterations=max_iterations)
    d = trimesh.proximity.signed_distance(cad_mesh, aligned)
    rmse = float(np.sqrt(np.mean(d ** 2)))
    return {"transform": T, "part_pose": np.linalg.inv(T),
            "rmse_mm": rmse, "converged": rmse < 1.0}


def deviation(scan, cad_mesh, transform=None, tol_mm=0.5):
    """As-built vs CAD: signed distance of each (registered) scan point to the CAD surface.
    Pass the scan->cad transform from register_to_cad; returns a deviation summary + verdict."""
    pts = scan.points if isinstance(scan, Scan) else np.asarray(scan, float)
    if transform is not None:
        pts = trimesh.transform_points(pts, transform)
    d = np.abs(trimesh.proximity.signed_distance(cad_mesh, pts))
    return {"max_mm": round(float(d.max()), 3), "mean_mm": round(float(d.mean()), 3),
            "p95_mm": round(float(np.percentile(d, 95)), 3), "rms_mm": round(float(np.sqrt(np.mean(d**2))), 3),
            "in_tolerance": bool(d.max() <= tol_mm), "tol_mm": tol_mm,
            "n_out": int((d > tol_mm).sum())}


def matrix_to_pose(T_mm):
    """4x4 (mm) -> (xyz metres, wxyz quat) for tracking.world.Pose."""
    xyz_m = (np.asarray(T_mm)[:3, 3] / 1000.0)
    q = trimesh.transformations.quaternion_from_matrix(T_mm)   # wxyz
    return tuple(xyz_m), tuple(q)


def estimate_pose(scan, cad_mesh, initial=None):
    """Scan -> the part's 6-DoF pose (metres + wxyz), ready for tracking.observe()."""
    reg = register_to_cad(scan, cad_mesh, initial)
    xyz, quat = matrix_to_pose(reg["part_pose"])
    return {"xyz": xyz, "quat": quat, "rmse_mm": reg["rmse_mm"], "converged": reg["converged"]}


if __name__ == "__main__":
    cad = trimesh.load("exports/example_plate.stl")
    real = trimesh.transformations.rotation_matrix(np.radians(8), [0, 0, 1])
    real[:3, 3] = [4.0, -3.0, 1.5]                             # the part's real pose (mm)
    scan = simulate_scan(cad, pose_mm=real, noise_mm=0.05)
    reg = register_to_cad(scan, cad)
    est = matrix_to_pose(reg["part_pose"])
    print(f"scan: {len(scan.points)} pts (1 view, {scan.noise_mm}mm noise)")
    print(f"registered rmse {reg['rmse_mm']:.3f}mm, converged={reg['converged']}")
    print(f"recovered pose xyz(mm)={np.round(np.array(est[0])*1000,2).tolist()} "
          f"(real was [4.0, -3.0, 1.5])")
    print("as-built deviation:", deviation(scan, cad, reg["transform"]))
    bad = simulate_scan(cad, pose_mm=real, noise_mm=0.05, defect=([20, 0, 5], 8.0, 1.2))
    breg = register_to_cad(bad, cad)
    print("with a 1.2mm bump:", deviation(bad, cad, breg["transform"]))
