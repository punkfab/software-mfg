"""wirebender_cell.py — compose the wire bender's OWN forward model by reference.

The bender lives in ../wirebender (read-only sibling repo). Its sim/bend_model.py
walks a bending program into the resulting 3D wire polyline. We invoke it through
the cell's own interpreter (never importing/forking it, never writing to it —
PYTHONDONTWRITEBYTECODE=1), exactly like we compose its CAD in sync_cells.py.

This makes the orchestration graph's `bend_wire` op real: it produces the actual
bent wire and a principled cycle time, instead of a stub duration.
"""

import json
import os
import subprocess
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent

# A bending program: list of (op, value). Ops map 1:1 to the machine's 3 axes:
#   feed mm, rotate deg, bend deg.  This one makes a staple (two 90° corners).
PROGRAMS = {
    "staple": [("feed", 20.0), ("bend", 90.0), ("feed", 30.0), ("bend", 90.0), ("feed", 20.0)],
}

# Axis speeds for the cycle-time model (feed from the slicer's default 400 mm/min).
FEED_MM_S = 400.0 / 60.0
BEND_DEG_S = 60.0
ROTATE_DEG_S = 120.0
OP_OVERHEAD_S = 0.15

_RUNNER = r"""
import sys, os, json
import numpy as np
sys.path.insert(0, os.path.join(os.environ["WB_ROOT"], "sim"))
import bend_model as bm
prog = [(op, float(v)) for op, v in json.loads(os.environ["WB_PROG"])]
pts = np.asarray(bm.simulate(prog), float)
print("RESULT:" + json.dumps({"points": pts.tolist(), "length": float(bm.total_length(pts))}))
"""


def _wirebender():
    spec = yaml.safe_load((ROOT / "cells.yaml").read_text())["cells"]["wirebender"]
    root = (ROOT / spec["path"]).resolve()
    return root, root / spec["python"]


def bend_duration(program) -> float:
    """Cycle time (s) for a bending program from the axis-speed model."""
    t = 0.0
    for op, v in program:
        if op == "feed":
            t += abs(v) / FEED_MM_S
        elif op == "bend":
            t += abs(v) / BEND_DEG_S
        elif op == "rotate":
            t += abs(v) / ROTATE_DEG_S
        t += OP_OVERHEAD_S
    return round(t, 2)


def staple_in_workcell(center, z, program_name="staple"):
    """Place the bender's produced wire into the workcell: Nx3 points (metres),
    planar staple centred (centroid) at `center` xy and lifted to height `z` —
    the geometry the arm presents and the shear closes on. Closes the bender→arm
    geometric loop (the bender's real output becomes the arm's real input)."""
    import numpy as np
    pts = np.asarray(simulate(program_name)["points"], float) * 1e-3   # mm -> m
    pts = pts - pts.mean(0)
    pts[:, 0] += center[0]
    pts[:, 1] += center[1]
    pts[:, 2] = z
    return pts.tolist()


def simulate(program_name="staple"):
    """Run the bender's forward model by reference; return the produced wire."""
    program = PROGRAMS[program_name]
    root, py = _wirebender()
    if not py.exists():
        raise FileNotFoundError(f"wirebender interpreter not found: {py}")
    proc = subprocess.run(
        [str(py), "-c", _RUNNER], cwd=str(root), capture_output=True, text=True,
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1",
             "WB_ROOT": str(root), "WB_PROG": json.dumps(program)},
    )
    line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT:")), None)
    if line is None:
        raise RuntimeError(f"bend_model failed:\n{proc.stdout[-500:]}\n{proc.stderr[-500:]}")
    out = json.loads(line[len("RESULT:"):])
    n_bends = sum(1 for op, _ in program if op == "bend")
    return {"points": out["points"], "length": round(out["length"], 2),
            "n_bends": n_bends, "duration": bend_duration(program), "program": program_name}
