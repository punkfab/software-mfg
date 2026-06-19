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


def build_job() -> OperationGraph:
    return OperationGraph([
        # printer branch (the slow one)
        Operation("print_bracket", "printer", 40.0),
        Operation("eject_bracket", "printer", 7.0, needs=("print_bracket",), action=_eject_action),
        # wire branch — the Phase-3 form -> present -> shear chain
        Operation("bend_wire", "bender", 8.0),
        Operation("present_wire", "arm", 3.0, needs=("bend_wire",), tool="shear"),
        Operation("shear_wire", "arm", 2.0, needs=("present_wire",), tool="shear", action=_cut_action),
        # join: insert the sheared wire into the ejected bracket
        Operation("assemble", "arm", 5.0, needs=("eject_bracket", "shear_wire")),
    ])
