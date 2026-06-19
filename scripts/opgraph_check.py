#!/usr/bin/env python3
"""Phase 3 gate: the scheduler produces a correct, overlapping schedule.

Pure logic (no sim). Asserts the schedule respects precedence and resource
exclusivity, covers every op, and beats the sequential baseline (overlap win).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "orchestration"))
from job import build_job  # noqa: E402
from opgraph import schedule, sequential_makespan  # noqa: E402

EPS = 1e-9


def main() -> int:
    g = build_job()
    sched, makespan = schedule(g)
    seq = sequential_makespan(g)
    problems = []

    if set(sched) != set(g.ops):
        problems.append("not every op was scheduled")

    # precedence: an op starts no earlier than every prerequisite finishes
    for name, op in g.ops.items():
        s, _f, _r = sched[name]
        for n in op.needs:
            if s + EPS < sched[n][1]:
                problems.append(f"{name} starts {s:.1f} before {n} finishes {sched[n][1]:.1f}")

    # resource exclusivity: one op per resource at a time
    by_res = {}
    for name, (s, f, r) in sched.items():
        by_res.setdefault(r, []).append((s, f, name))
    for r, items in by_res.items():
        items.sort()
        for (s1, f1, n1), (s2, f2, n2) in zip(items, items[1:]):
            if s2 + EPS < f1:
                problems.append(f"resource {r}: {n1} and {n2} overlap")

    if makespan >= seq:
        problems.append(f"no overlap win (scheduled {makespan:.1f} >= sequential {seq:.1f})")

    saved = seq - makespan
    print(f"ops={len(g.ops)} resources={len(by_res)}  "
          f"sequential={seq:.1f}s scheduled={makespan:.1f}s saved={saved:.1f}s ({100*saved/seq:.0f}%)")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: schedule respects precedence + resource limits and overlaps to cut cycle time")
    return 0


if __name__ == "__main__":
    sys.exit(main())
