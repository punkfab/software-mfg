#!/usr/bin/env python3
"""Gate: the end-to-end CAD->plan->CAM->motion->track->verify pipeline runs.

One sample assembly must: schedule, turn its glue op into a reachable toolpath (IK
residual small), track each part's pose through carry (drift) and re-observation, and
verify every placement against the CAD-nominal within tolerance. This is the backbone
gate — every stage of the assembly pipeline exercised together.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in ("", "orchestration", "sim"):
    p = str(ROOT / sub) if sub else str(ROOT)
    sys.path.insert(0, p)
from assemble import run  # noqa: E402
from tracking import FRESH  # noqa: E402


def main() -> int:
    rep = run("seam")
    problems = []

    if rep["makespan_s"] <= 0:
        problems.append("plan did not schedule")
    if not rep["reachable"]:
        problems.append(f"glue toolpath out of reach (IK residual {rep['ik_residual_mm']}mm)")
    for name, s in rep["staleness"].items():
        if s["before_observe"]["verdict"] == FRESH:
            problems.append(f"{name}: pose did not degrade while carried unobserved")
        if s["after_observe"]["verdict"] != FRESH:
            problems.append(f"{name}: observation did not re-anchor pose")
    if not rep["verify_ok"]:
        problems.append(f"assembly verification failed: {rep['verify']}")

    print(f"assemble '{rep['program']}': makespan {rep['makespan_s']}s | "
          f"toolpath {rep['toolpath_points']}pts, IK residual {rep['ik_residual_mm']}mm "
          f"({'reachable' if rep['reachable'] else 'OUT OF REACH'})")
    for name, s in rep["staleness"].items():
        print(f"  {name}: carry -> {s['before_observe']['verdict']} -> observe -> "
              f"{s['after_observe']['verdict']} | placed {rep['verify'][name].get('trans_mm')}mm off nominal")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: CAD -> plan -> CAM toolpath -> motion (IK) -> tracked -> verified vs CAD")
    return 0


if __name__ == "__main__":
    sys.exit(main())
