"""coord_job.py — a two-arm coordination task: glue-and-hold.

The first task that genuinely needs the leader/follower pair. One arm (arm_hold)
grips and presents a part at a seam; the other (arm_glue) carries the hot-glue tool
and lays a bead; then the joint must be HELD until the adhesive sets. A single arm
cannot do it — holding and gluing must happen at the same time, and the hold must
span the whole apply+set window. This is the coordination constraint the scheduler
has to respect, and coord_check validates it.

The melt cartridge is PRE-HEATED at the dock (glue_cell.ready_s, ~2 min) before the
sequence — that long thermal cost is paid once and amortized across many joints, so
it is not on the coordinated critical path (a hot tool arrives ready).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in ("orchestration", "sim"):
    p = str(ROOT / sub)
    if p not in sys.path:
        sys.path.insert(0, p)
from opgraph import Operation, OperationGraph, schedule  # noqa: E402
import glue_cell  # noqa: E402

POS_S = 4.0        # grip + present the part at the seam
APPROACH_S = 3.0   # bring the (already hot) tool to the bead start
REL_S = 2.0        # release once set

CAPS = {"arm_hold": 1, "arm_glue": 1, "cure": 1}


def build_graph(program="seam"):
    r = glue_cell.apply_bead(program)
    apply_s, set_s = r["apply_s"], r["set_s"]
    ops = [
        Operation("position", "arm_hold", POS_S),
        Operation("approach", "arm_glue", APPROACH_S),
        # the bead needs BOTH the part presented and the hot tool in place
        Operation("apply", "arm_glue", apply_s, needs=("position", "approach"), tool="glue"),
        # the holding arm keeps gripping through apply + set (its whole commitment)
        Operation("hold_through", "arm_hold", round(apply_s + set_s, 2), needs=("position",)),
        # the adhesive sets (passive resource, not an arm)
        Operation("set_dwell", "cure", set_s, needs=("apply",)),
        # release only after the hold is done AND the joint has set
        Operation("release", "arm_hold", REL_S, needs=("hold_through", "set_dwell")),
    ]
    return OperationGraph(ops), r


def schedule_job(program="seam"):
    g, r = build_graph(program)
    sched, mk = schedule(g, capacity=CAPS)
    return sched, mk, r


if __name__ == "__main__":
    from opgraph import ascii_gantt
    sched, mk, r = schedule_job("perimeter")
    print(f"glue-and-hold ({r['program']}): makespan {mk:.1f}s, "
          f"hold commitment {r['hold_window_s']}s, preheat {r['ready_s']}s (amortized)")
    print(ascii_gantt(sched, mk))
