"""job.py — a concrete multi-cell assembly job as an operation graph.

Produces one unit: a bent + sheared wire staple inserted into a printed bracket.
Spans three resources/cells — printer, bender, arm — so the scheduler can overlap
the slow print with the wire work. Two ops carry real sim actions (the printer
eject and the arm tool-change + shear); the rest are timed stubs until their
cells are wired in (wirebender bend, the assembly insert).

Embeds the Phase-3 milestone chain explicitly: bend_wire -> present_wire ->
shear_wire (form -> present -> shear).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
for sub in ("orchestration", "sim", "scripts"):
    sys.path.insert(0, str(ROOT / sub))

from opgraph import Operation, OperationGraph  # noqa: E402
import wirebender_cell  # noqa: E402

# bend_wire's duration comes from the bender's own program (pure time model, no
# subprocess); its action runs the bender's real forward model when executed.
BEND_SECS = wirebender_cell.bend_duration(wirebender_cell.PROGRAMS["staple"])


def _bend_action():
    r = wirebender_cell.simulate("staple")
    return f"wire bent: {r['length']} mm, {r['n_bends']} bends ({r['duration']}s)"


def _eject_action():
    import mujoco
    from printer_cell import build_model, run_eject
    m = build_model()
    run_eject(m, mujoco.MjData(m))
    return "bracket ejected"


def _cut_action():
    from toolchange_check import run as run_toolchange   # couple->present->shear->return
    run_toolchange()
    return "wire sheared"


def _unit_ops(suffix="", with_actions=True):
    """One unit's operations. `suffix` makes names unique across pipeline units."""
    p, e, b, pr, sh, asm = (f"{n}{suffix}" for n in
                            ("print_bracket", "eject_bracket", "bend_wire",
                             "present_wire", "shear_wire", "assemble"))
    return [
        # printer branch (the slow one)
        Operation(p, "printer", 40.0),
        Operation(e, "printer", 7.0, needs=(p,), action=_eject_action if with_actions else None),
        # wire branch — the Phase-3 form -> present -> shear chain.
        # bend_wire now runs the wire bender's OWN forward model (composed by reference).
        Operation(b, "bender", BEND_SECS, action=_bend_action if with_actions else None),
        Operation(pr, "arm", 3.0, needs=(b,), tool="shear"),
        Operation(sh, "arm", 2.0, needs=(pr,), tool="shear", action=_cut_action if with_actions else None),
        # join: insert the sheared wire into the ejected bracket
        Operation(asm, "arm", 5.0, needs=(e, sh)),
    ]


def build_job() -> OperationGraph:
    """A single unit (carries the real sim actions)."""
    return OperationGraph(_unit_ops())


def build_pipeline(n_units: int) -> OperationGraph:
    """N independent units sharing the resources — the production pipeline."""
    ops = []
    for i in range(n_units):
        ops += _unit_ops(suffix=f"_u{i}", with_actions=False)
    return OperationGraph(ops)


PER_UNIT_PRINTER = 47.0   # printer busy-time per unit (print 40 + eject 7) = the bottleneck
