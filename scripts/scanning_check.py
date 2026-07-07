#!/usr/bin/env python3
"""Gate: 3D scanning is a sound reality-capture leg — scan -> CAD -> pose + deviation.

Asserts the two jobs scanning does for this repo, plus the honest failure mode:
  - MULTI-VIEW scan registers to CAD and recovers a known 6-DoF pose within tolerance;
  - a SINGLE flat view is DEGENERATE (recovers a wrong pose despite fitting) — coverage,
    not just a low residual, is what makes a scan trustworthy;
  - a faithful scan reads in-tolerance vs CAD; an injected bump is flagged as as-built
    deviation (the geometry version of the calibration writeback);
  - a scan-derived pose feeds tracking.observe() and re-anchors the part to FRESH (scanning
    is the real observation source behind the world model).
"""

import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))
import scanning as sc  # noqa: E402
from tracking.world import Pose, WorldModel  # noqa: E402

CAD = trimesh.load(str(ROOT / "exports" / "example_plate.stl"))


def main() -> int:
    problems = []
    real = trimesh.transformations.rotation_matrix(np.radians(8), [0, 0, 1])
    real[:3, 3] = [4.0, -3.0, 1.5]                      # the part's real pose (mm)

    # 1. multi-view recovers the pose
    scan = sc.simulate_scan(CAD, pose_mm=real, noise_mm=0.05)
    est = sc.estimate_pose(scan, CAD)
    terr = np.linalg.norm(np.array(est["xyz"]) * 1000 - real[:3, 3])
    if terr > 0.6 or not est["converged"]:
        problems.append(f"multi-view pose off by {terr:.2f}mm (rmse {est['rmse_mm']})")

    # 2. single flat view is degenerate — coverage matters, not just residual
    s1 = sc.simulate_scan(CAD, pose_mm=real, noise_mm=0.05, view_dirs=sc.VIEWS_TOP)
    e1 = sc.estimate_pose(s1, CAD)
    terr1 = np.linalg.norm(np.array(e1["xyz"]) * 1000 - real[:3, 3])
    if terr1 < 1.0:
        problems.append("single flat view should be degenerate (large pose error), but wasn't")
    if terr >= terr1:
        problems.append("multi-view should beat single-view on pose accuracy")

    # 3. as-built deviation: faithful in tolerance, injected bump flagged
    reg = sc.register_to_cad(scan, CAD)
    dev = sc.deviation(scan, CAD, reg["transform"], tol_mm=0.5)
    if not dev["in_tolerance"] or dev["n_out"] > 0:
        problems.append(f"faithful scan wrongly out of tolerance: {dev}")
    bad = sc.simulate_scan(CAD, pose_mm=real, defect=([20, 0, 5], 8.0, 1.2))
    bdev = sc.deviation(bad, CAD, sc.register_to_cad(bad, CAD)["transform"], tol_mm=0.5)
    if bdev["in_tolerance"] or bdev["n_out"] == 0 or bdev["max_mm"] < 0.8:
        problems.append(f"1.2mm bump not flagged as deviation: {bdev}")

    # 4. integration: a scan re-anchors a tracked part to FRESH
    w = WorldModel()
    w.add("plate", "parts/example_plate.py", Pose((0.0, 0.0, 0.0)))
    for _ in range(6):                                  # drift it un-observed
        w.tick(); w.carry("plate", w.parts["plate"].est)
    stale_before = w.staleness("plate")["verdict"]
    obs = sc.estimate_pose(scan, CAD)                   # the scan is the observation
    w.observe("plate", Pose(obs["xyz"], obs["quat"]))
    fresh_after = w.staleness("plate")["verdict"]
    if stale_before == "FRESH":
        problems.append("part should have drifted before the scan")
    if fresh_after != "FRESH":
        problems.append("scan observation should re-anchor the part to FRESH")

    # --- report ---
    print(f"multi-view ({scan.n_view} views): pose err {terr:.2f}mm, rmse {est['rmse_mm']}mm -> recovered")
    print(f"single view: pose err {terr1:.2f}mm (degenerate — low residual, wrong pose)")
    print(f"as-built: faithful max {dev['max_mm']}mm (in tol), 1.2mm bump -> max {bdev['max_mm']}mm, "
          f"{bdev['n_out']} pts out (FLAGGED)")
    print(f"tracking: carry -> {stale_before} -> scan.observe -> {fresh_after}")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: scan->CAD recovers pose (multi-view) + flags as-built deviation + re-anchors tracking")
    return 0


if __name__ == "__main__":
    sys.exit(main())
