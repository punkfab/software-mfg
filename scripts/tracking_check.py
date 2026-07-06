#!/usr/bin/env python3
"""Gate: the world model tracks part pose with staleness and verifies against CAD.

Pose is treated like a calibrated parameter: a grasped part carried without being
observed drifts to EXTRAPOLATING; an observation re-anchors it to FRESH; a placement is
checked against the CAD-nominal pose. This validates the tracking layer that turns a
plan into a *verified* assembly.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from tracking import FRESH, Pose, load_assembly, verify  # noqa: E402


def main() -> int:
    w = load_assembly()
    problems = []

    # grasp the staple and carry it, unobserved, until the estimate is untrustworthy
    w.tick(); w.grasp("staple", "arm_hold")
    for _ in range(6):
        w.tick(); w.carry("staple", w.parts["staple"].est)
    drifted = w.staleness("staple")
    if drifted["verdict"] == FRESH:
        problems.append(f"pose did not degrade while carried unobserved ({drifted})")

    # re-observe near the nominal -> re-anchored to FRESH, correction reported
    nom = w.parts["staple"].nominal
    corr = w.observe("staple", Pose((nom.xyz[0] + 0.0007, nom.xyz[1], nom.xyz[2]), nom.quat))
    anchored = w.staleness("staple")
    if anchored["verdict"] != FRESH:
        problems.append(f"observation did not re-anchor pose ({anchored})")
    w.place("staple"); w.place("bracket")

    # verify every placed part against the CAD-nominal pose
    ok, results = verify(w)
    if not ok:
        problems.append(f"assembly verification failed: {results}")

    print(f"track staple: carry -> {drifted['verdict']} (σ{drifted['sigma_mm']}mm) -> "
          f"observe (corrected {corr:.2f}mm) -> {anchored['verdict']}")
    print(f"verify vs CAD nominal: {'OK' if ok else 'FAIL'} "
          f"staple {results['staple'].get('trans_mm')}mm / {results['staple'].get('ang_deg')}deg")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: pose tracked with staleness (drift -> re-anchor), placements verified vs CAD")
    return 0


if __name__ == "__main__":
    sys.exit(main())
