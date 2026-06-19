#!/usr/bin/env python3
"""Schedule the multi-cell job and report the cycle time + Gantt.

Pass --execute to actually run the sim actions (printer eject, arm shear) in
scheduled order, proving the ops are real and not just durations.

    python scripts/opgraph_run.py [--execute]
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "orchestration"))
from job import build_job  # noqa: E402
from opgraph import ascii_gantt, schedule, sequential_makespan  # noqa: E402


def main() -> int:
    g = build_job()
    sched, makespan = schedule(g)
    seq = sequential_makespan(g)
    saved = seq - makespan

    print("OPERATION GRAPH — one unit (bracket + sheared wire)\n")
    print(ascii_gantt(sched, makespan))
    print()
    print(f"sequential (no overlap): {seq:5.1f}s")
    print(f"scheduled  (overlapped): {makespan:5.1f}s")
    print(f"saved by scheduling:     {saved:5.1f}s  ({100*saved/seq:.0f}% faster)")

    crit = max(sched.items(), key=lambda kv: kv[1][1])
    print(f"cycle time gated by:     {crit[0]} finishing at {crit[1][1]:.1f}s")

    if "--execute" in sys.argv:
        print("\nexecuting real sim actions in scheduled order ...")
        for name, (s, f, r) in sorted(sched.items(), key=lambda kv: kv[1][0]):
            op = g.ops[name]
            if op.action is not None:
                result = op.action()
                print(f"  [{s:5.1f}s] {r:7} {name:14} -> {result}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
