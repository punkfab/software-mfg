#!/usr/bin/env python3
"""Gate: the material-handling floor plans safe station moves + material flow.

A station is a solid on the floor, so a base move is a swept-solid interference check
(reusing sim/interference.py). This gate proves the floor:
  - flags overlapping footprints (a bad layout / blocked dock),
  - answers the work-envelope reach question (is a payload in an arm's reach?),
  - REFUSES a base move that would foul another station, naming the blocker,
  - ROUTES a material carrier around obstacles into an arm's envelope (the staging flow),
  - tracks base-pose staleness (odometry drift -> STALE/EXTRAPOLATING -> fiducial re-anchor).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))
from station import EXTRAPOLATING, FRESH, Floor, Station  # noqa: E402


def build_floor():
    return (Floor()
            .add(Station("arm", "arm", (0.0, 0.0, 0.0), footprint=(0.30, 0.30),
                         mobile=False, reach_m=0.50))
            .add(Station("fixture", "fixture", (0.40, 0.0, 0.0), footprint=(0.25, 0.25),
                         mobile=False, payload="bracket"))
            .add(Station("bin", "bin", (0.0, 1.20, 0.0), footprint=(0.30, 0.30),
                         payload="M3_screws"))
            .add(Station("blocker", "fixture", (0.0, 0.60, 0.0), footprint=(0.40, 0.40),
                         mobile=False)))


def main() -> int:
    problems = []
    f = build_floor()

    # 1. clean layout — nothing overlaps
    if f.clashes():
        problems.append(f"clean layout reported clashes: {f.clashes()}")

    # 2. work envelope — the arm reaches the fixture beside it, not the bin across the floor
    if not f.reachable("arm", "fixture")["reachable"]:
        problems.append("arm should reach the adjacent fixture")
    if f.reachable("arm", "bin")["reachable"]:
        problems.append("arm should NOT reach the bin 1.2m away")

    # 3. a base move straight through the blocker must be refused, naming the blocker
    blocked = f.move("bin", (0.0, 0.0, 0.0))
    if blocked["ok"]:
        problems.append("move straight through the blocker was allowed")
    elif not blocked.get("blocked_by") or blocked["blocked_by"]["part"] != "blocker":
        problems.append(f"blocked move did not name the blocker: {blocked}")

    # 4. staging flow — route the bin AROUND the blocker into the arm's envelope
    stage = f.stage_into_envelope("bin", "arm", dock_pose=(-0.42, 0.0, 0.0),
                                  via=[(-0.80, 1.20, 0.0), (-0.80, 0.0, 0.0)])
    if not stage["ok"]:
        problems.append(f"routed staging into the envelope failed: {stage}")
    if stage.get("payload") != "M3_screws":
        problems.append("staged carrier lost its payload identity")

    # 5. base-pose staleness — drifted over the long route, re-anchored by a fiducial
    drifted = f.staleness("bin")["verdict"]
    if drifted == FRESH:
        problems.append("base pose should have drifted (not FRESH) after a long un-fixed route")
    fixed = f.fix("bin")
    if fixed["verdict"] != FRESH:
        problems.append("fiducial fix should re-anchor the base pose to FRESH")

    # 6. overlap is a clash
    f.add(Station("oops", "bin", (0.40, 0.0, 0.0), footprint=(0.25, 0.25)))
    pairs = [c["pair"] for c in f.clashes()]
    if not any("oops" in p for p in pairs):
        problems.append(f"overlapping footprint not flagged: {pairs}")

    # --- report ---
    print(f"floor: arm reaches fixture @{f.reachable('arm','fixture')['distance_m']}m "
          f"(reach {f.stations['arm'].reach_m}m), not the bin @1.2m")
    print(f"move through blocker: REFUSED (fouls {blocked.get('blocked_by',{}).get('part')})")
    print(f"staged '{stage.get('payload')}' around the blocker in "
          f"{len(stage.get('route',{}).get('legs',[]))} legs -> in envelope "
          f"({stage['reach']['distance_m']}m); base pose {drifted} -> fix -> {fixed['verdict']}")
    print(f"overlap clash flagged: {[p for p in pairs if 'oops' in p]}")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: floor plans collision-checked station moves + routes material into the work envelope")
    return 0


if __name__ == "__main__":
    sys.exit(main())
