#!/usr/bin/env python3
"""Gate: the two-arm glue-and-hold task is correctly coordinated.

The holding arm must grip the joint CONTINUOUSLY from present through release (never
drop the part), the release must wait until the adhesive has set, and the task must
genuinely require two arms (the glue and the hold overlap in time — one arm can't do
both). This is the first multi-arm coordination gate.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "orchestration"))
sys.path.insert(0, str(ROOT / "sim"))
from opgraph import ascii_gantt  # noqa: E402
from coord_job import schedule_job  # noqa: E402

EPS = 1e-6


def overlaps(a, b):
    return a[0] < b[1] - EPS and b[0] < a[1] - EPS


def main() -> int:
    sched, mk, r = schedule_job("perimeter")
    problems = []

    # arm_hold must be continuously busy (no gap = the part is never dropped)
    hold_iv = sorted((s, f) for _, (s, f, res) in sched.items() if res == "arm_hold")
    for i in range(1, len(hold_iv)):
        gap = hold_iv[i][0] - hold_iv[i - 1][1]
        if gap > 1e-3:
            problems.append(f"holding arm drops the part for {gap:.1f}s (gap in grip)")

    # release must not begin until the joint has set
    rel_s = sched["release"][0]
    set_f = sched["set_dwell"][1]
    if rel_s < set_f - 1e-3:
        problems.append(f"released {set_f - rel_s:.1f}s before the glue set")

    # the task must REQUIRE two arms: apply (arm_glue) overlaps hold_through (arm_hold)
    apply_iv = sched["apply"][:2]
    holdt_iv = sched["hold_through"][:2]
    if not overlaps(apply_iv, holdt_iv):
        problems.append("glue and hold do not overlap — task would not need two arms")

    grip_span = hold_iv[-1][1] - hold_iv[0][0]
    print(f"glue-and-hold ({r['program']}): makespan {mk:.1f}s | holding arm gripped "
          f"{grip_span:.1f}s continuously | release after set OK | needs 2 arms (glue∥hold)")
    print(f"  preheat {r['ready_s']}s is a one-time dock cost (amortized across joints)")
    print(ascii_gantt(sched, mk))
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: holding arm never drops the part, releases only after set, and the task "
          "requires both arms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
