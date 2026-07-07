"""interference.py — solid part interference / clearance checking.

The question CAM can't answer from a toolpath alone: **does the moving solid hit
anything?** A part on its way into a seat, a tool sweeping a bead — either can foul a
fixture or a neighbouring part. This module checks real triangle meshes (the build123d
STL exports, mm) for overlap, and sweeps a moving mesh along a path to find the first
collision.

Self-contained: no python-fcl. Two-phase, the standard collision pipeline —
  BROAD:  axis-aligned bounding-box (AABB) overlap prunes the non-colliding pairs cheap.
  NARROW: for pairs whose AABBs overlap, a manifold3d boolean **intersection volume** is
          the ground truth (same engine as the CAD booleans). Volume > a noise floor is a
          real interference.

A collision checker must never *silently* under-report. manifold needs closed volumes,
but real CAD exports are sometimes non-watertight (coincident-face seams at booleans). So
`_as_volume` repairs where it can and otherwise falls back to the **convex hull** — which
is conservative (it can only over-report a collision, never miss one) — and every result
carries an `approximate` flag when a hull proxy was used. Fail toward flagging, never
toward silence.

Units are millimetres throughout (matching the STL exports). Poses from the tracking
world model are metres + wxyz quaternion — `from_world_pose()` converts.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

VOL_TOL_MM3 = 1.0        # boolean intersection below this is meshing noise, not a collision
NEAR_MM = 2.0            # report min-separation for non-touching pairs closer than this
_BROAD_MARGIN_MM = 0.5   # AABB broad-phase slop so we never skip a true narrow-phase hit


@functools.lru_cache(maxsize=64)
def load_mesh(path: str) -> trimesh.Trimesh:
    """Load an STL (cached). Returns a single Trimesh (mm, as build123d exports)."""
    m = trimesh.load(str(path), force="mesh")
    if not isinstance(m, trimesh.Trimesh):
        m = m.dump(concatenate=True)
    return m


def _as_mesh(x) -> trimesh.Trimesh:
    if isinstance(x, trimesh.Trimesh):
        return x
    return load_mesh(str(x))


def _as_volume(m: trimesh.Trimesh):
    """Return (closed_mesh, exact) suitable for a boolean. Watertight meshes pass through
    (exact). Open meshes are repaired if cheap repair closes them (exact), else replaced by
    their convex hull (approximate — conservative, over-reports rather than misses)."""
    if m.is_watertight:
        return m, True
    r = m.copy()
    r.merge_vertices()
    r.process(validate=True)
    trimesh.repair.fill_holes(r)
    if r.is_watertight:
        return r, True
    return m.convex_hull, False


def pose_matrix_mm(xyz_mm=(0, 0, 0), quat_wxyz=(1, 0, 0, 0)) -> np.ndarray:
    """4x4 rigid transform (mm) from a translation (mm) + wxyz quaternion."""
    T = trimesh.transformations.quaternion_matrix(np.asarray(quat_wxyz, float))
    T[:3, 3] = np.asarray(xyz_mm, float)
    return T


def from_world_pose(pose) -> np.ndarray:
    """4x4 transform (mm) from a tracking.world.Pose (xyz in metres, wxyz quat)."""
    return pose_matrix_mm(np.asarray(pose.xyz, float) * 1000.0, pose.quat)


def aabb_overlap(a: trimesh.Trimesh, b: trimesh.Trimesh, margin=0.0) -> bool:
    """Do two meshes' axis-aligned bounding boxes overlap (broad phase)?"""
    (alo, ahi), (blo, bhi) = a.bounds, b.bounds
    return bool(np.all(alo - margin <= bhi) and np.all(blo - margin <= ahi))


def _intersection(va: trimesh.Trimesh, vb: trimesh.Trimesh):
    """Solid intersection of two *volumes*; None if disjoint. AABB-pruned."""
    if not aabb_overlap(va, vb, _BROAD_MARGIN_MM):
        return None
    try:
        inter = trimesh.boolean.intersection([va, vb], engine="manifold")
    except Exception:
        return None
    if inter is None or inter.is_empty or len(inter.vertices) == 0 or abs(inter.volume) <= 0:
        return None
    return inter


def intersection_volume(a, b) -> float:
    """Volume (mm^3) of the solid overlap between two meshes; 0.0 if disjoint."""
    va, _ = _as_volume(_as_mesh(a))
    vb, _ = _as_volume(_as_mesh(b))
    inter = _intersection(va, vb)
    return 0.0 if inter is None else abs(float(inter.volume))


def min_separation_mm(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    """Closest approach (mm) between two non-overlapping meshes, vertex-sampled from the
    smaller mesh onto the other's surface. Approximate (misses edge-edge minima), but
    honest for a clearance warning. 0.0 if they overlap."""
    small, big = (a, b) if len(a.vertices) <= len(b.vertices) else (b, a)
    _, dist, _ = trimesh.proximity.closest_point(big, small.vertices)
    return float(np.min(dist))


@dataclass
class Scene:
    """A set of named, placed solids (world mm). Each body's boolean-ready volume + its
    exactness flag are computed once at placement, so pairwise/sweep tests are cheap."""
    bodies: dict = field(default_factory=dict)     # name -> Trimesh (world-placed, mm)
    _vol: dict = field(default_factory=dict)       # name -> (volume Trimesh, exact bool)

    def place(self, name: str, mesh, transform_mm=None):
        m = _as_mesh(mesh).copy()
        if transform_mm is not None:
            m.apply_transform(np.asarray(transform_mm, float))
        self.bodies[name] = m
        self._vol[name] = _as_volume(m)
        return self

    def interferences(self, vol_tol_mm3=VOL_TOL_MM3):
        """Every pair of bodies that solidly overlap, worst first. `depth_mm` is the
        thinnest extent of the overlap solid (how far the parts interpenetrate)."""
        names, hits = list(self.bodies), []
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                na, nb = names[i], names[j]
                (va, ea), (vb, eb) = self._vol[na], self._vol[nb]
                inter = _intersection(va, vb)
                if inter is None:
                    continue
                v = abs(float(inter.volume))
                if v <= vol_tol_mm3:
                    continue
                depth = float(np.min(inter.bounds[1] - inter.bounds[0]))
                hits.append({"pair": (na, nb), "volume_mm3": round(v, 3),
                             "depth_mm": round(depth, 3), "approximate": not (ea and eb)})
        hits.sort(key=lambda h: -h["volume_mm3"])
        return hits

    def clearances(self, near_mm=NEAR_MM, vol_tol_mm3=VOL_TOL_MM3):
        """Non-touching pairs whose closest approach is under near_mm (near-miss watch)."""
        names, out = list(self.bodies), []
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = self.bodies[names[i]], self.bodies[names[j]]
                if not aabb_overlap(a, b, near_mm):
                    continue
                va, vb = self._vol[names[i]][0], self._vol[names[j]][0]
                if _intersection(va, vb) is not None:
                    continue                    # overlapping -> an interference, not a clearance
                sep = min_separation_mm(a, b)
                if sep <= near_mm:
                    out.append({"pair": (names[i], names[j]), "clearance_mm": round(sep, 3)})
        out.sort(key=lambda c: c["clearance_mm"])
        return out

    def check(self, near_mm=NEAR_MM, vol_tol_mm3=VOL_TOL_MM3):
        """Static report: solid overlaps (fail) + near-miss clearances (warn)."""
        inter = self.interferences(vol_tol_mm3)
        return {"ok": len(inter) == 0,
                "approximate": any(h["approximate"] for h in inter),
                "interferences": inter,
                "clearances": self.clearances(near_mm, vol_tol_mm3)}


def sweep(moving, transforms_mm, statics: Scene, ignore=(), vol_tol_mm3=VOL_TOL_MM3):
    """Move `moving` (a mesh, local mm) through each transform and test it against every
    static body. Returns the FIRST colliding waypoint (motion should stop before it).

    transforms_mm: list of 4x4 world transforms for the moving mesh (one per waypoint).
    ignore: static body names to skip (e.g. the part currently grasped).

    Result: {ok, approximate, first_hit: {index, part, volume_mm3} | None, waypoints,
    tested_pairs, pruned} — `pruned` counts AABB broad-phase skips (the cheap majority)."""
    vbase, exact_moving = _as_volume(_as_mesh(moving))
    statics_list = [(n, self_vol, exact) for n, (self_vol, exact) in statics._vol.items()
                    if n not in ignore]
    approximate = not exact_moving
    tested = pruned = 0
    for idx, T in enumerate(transforms_mm):
        mv = vbase.copy()
        mv.apply_transform(np.asarray(T, float))
        for name, sm, exact in statics_list:
            if not aabb_overlap(mv, sm, _BROAD_MARGIN_MM):
                pruned += 1
                continue
            tested += 1
            inter = _intersection(mv, sm)
            if inter is not None and abs(float(inter.volume)) > vol_tol_mm3:
                return {"ok": False, "approximate": approximate or not exact,
                        "first_hit": {"index": idx, "part": name,
                                      "volume_mm3": round(abs(float(inter.volume)), 3)},
                        "waypoints": len(transforms_mm), "tested_pairs": tested, "pruned": pruned}
    return {"ok": True, "approximate": approximate, "first_hit": None,
            "waypoints": len(transforms_mm), "tested_pairs": tested, "pruned": pruned}


def transforms_along(poses_m, quat_wxyz=(1, 0, 0, 0), tool_offset_mm=(0, 0, 0)):
    """Build sweep transforms (mm) from a list of tip poses (metres, from a Toolpath).
    tool_offset_mm shifts the moving mesh's own origin onto the tip (its TCP)."""
    off = np.asarray(tool_offset_mm, float)
    return [pose_matrix_mm(np.asarray(p, float) * 1000.0 + off, quat_wxyz) for p in poses_m]
