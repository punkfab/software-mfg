"""toolpath.py — CAM: a Cartesian tool path + the motion bridge to joint space.

A Toolpath is an ordered list of tool-tip targets (the CAM output). One generator per
operation — `bead_toolpath` (dispense along a seam), `insert_toolpath` (plunge for a
press / peg-in-hole), `pick_place_toolpath` (transport with a lift) — each returns the
same (poses + per-point normals) shape, so motion and interference checking are uniform
across operations. `to_joint_traj` runs each target through an IK solver to joint
waypoints (motion control); `interference.transforms_along` turns the same poses into a
swept collision check.

The solver is PLUGGABLE. The built-in positional DLS IK (sim/ik.py) keeps the gates
self-contained; **Placo** (QP, orientation-aware) drops in for real toolpath following,
where the nozzle must stay normal to the surface along the whole path — the exact case
positional IK can't express. Same fallback discipline as the calibration reads: a
built-in default so it always runs, a richer backend when it's available.
"""

import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))

STEP_MM = 10.0   # resample the path to at most this spacing (denser -> smoother motion)


@dataclass
class Toolpath:
    poses: list                     # list of (x, y, z) tip targets (world, metres)
    label: str = ""
    normals: list = field(default_factory=list)   # per-point surface normal (for Placo)


def _resample(points_mm, step_mm=STEP_MM):
    """Resample a polyline (mm) so consecutive points are <= step_mm apart."""
    out = [points_mm[0]]
    for a, b in zip(points_mm[:-1], points_mm[1:]):
        a, b = np.array(a, float), np.array(b, float)
        seg = float(np.linalg.norm(b - a))
        n = max(1, int(np.ceil(seg / step_mm)))
        for i in range(1, n + 1):
            out.append(tuple(a + (b - a) * i / n))
    return out


def bead_toolpath(bead_points_mm, origin_xyz, label="bead", step_mm=STEP_MM):
    """Map a 2D bead path (mm, in the seam plane) onto the work surface at origin_xyz.
    Centres the path on the origin (keeps it inside the arm's reach) and resamples."""
    pts = _resample(bead_points_mm, step_mm)
    c = np.mean(np.array(pts, float), axis=0)          # centre on the origin
    o = np.array(origin_xyz, float)
    poses = [tuple(o + np.array([(px - c[0]) / 1000.0, (py - c[1]) / 1000.0, 0.0]))
             for (px, py) in pts]
    normals = [(0.0, 0.0, 1.0)] * len(poses)           # bead laid down onto a flat surface
    return Toolpath(poses, label, normals)


def insert_toolpath(seat_xyz, approach_mm=25.0, axis=(0, 0, 1), label="insert", step_mm=STEP_MM):
    """A straight plunge for press / peg-in-hole / bearing-seat ops: descend along -axis
    from approach_mm above the seat down to the seat. Normals point back up +axis (the
    insertion direction the tool holds). Feeds interference.sweep for a clearance check."""
    ax = np.asarray(axis, float)
    ax = ax / (np.linalg.norm(ax) or 1.0)
    seat = np.asarray(seat_xyz, float)
    start = seat + ax * (approach_mm / 1000.0)
    poses = _resample([tuple(start), tuple(seat)], step_mm)
    normals = [tuple(ax)] * len(poses)
    return Toolpath(poses, label, normals)


def pick_place_toolpath(pick_xyz, place_xyz, safe_z_mm=40.0, label="pick_place", step_mm=STEP_MM):
    """Transport a part pick -> place with a lift over obstacles: up to a safe plane,
    across, then straight down onto the target. The path the *carried part* sweeps for
    interference against the scene (ignore the part itself when checking)."""
    pick, place = np.asarray(pick_xyz, float), np.asarray(place_xyz, float)
    dz = safe_z_mm / 1000.0
    over_pick = pick + np.array([0.0, 0.0, dz])
    over_place = place + np.array([0.0, 0.0, dz])
    poses = _resample([tuple(pick), tuple(over_pick), tuple(over_place), tuple(place)], step_mm)
    normals = [(0.0, 0.0, 1.0)] * len(poses)
    return Toolpath(poses, label, normals)


def builtin_ik_solver():
    """Default motion solver: the workcell's positional DLS IK. Returns solve(target)."""
    import mujoco

    import ik
    from workcell import build_model

    m = build_model()
    d = mujoco.MjData(m)
    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")

    def solve(target):
        return ik.solve_ik(m, d, tip, np.asarray(target, float))

    return solve


def to_joint_traj(toolpath, solve=None):
    """Run the toolpath through IK -> joint waypoints. Returns (waypoints, max_residual_mm).

    Pass a Placo-backed `solve(target)->(q, err_m)` for orientation-aware following;
    defaults to the built-in positional solver."""
    solve = solve or builtin_ik_solver()
    q_wps, max_err = [], 0.0
    for tgt in toolpath.poses:
        q, err = solve(tgt)
        q_wps.append(q.tolist() if hasattr(q, "tolist") else list(q))
        max_err = max(max_err, err)
    return q_wps, max_err * 1000.0
