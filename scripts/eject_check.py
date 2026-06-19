#!/usr/bin/env python3
"""Phase gate: eject-in-place actually removes the part.

Runs the printer-cell eject sequence and asserts the part starts on the bed and
ends ejected past the front edge and down in the catch bin.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from printer_cell import BED, BED_TOP_Z, FRONT_Y, build_model, run_eject  # noqa: E402


def main() -> int:
    m = build_model()
    d = mujoco.MjData(m)
    part = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "part")

    mujoco.mj_forward(m, d)
    start = d.xpos[part].copy()
    on_bed = abs(start[2] - (BED_TOP_Z + 0.015)) < 0.006 and abs(start[0]) < BED / 2

    run_eject(m, d)
    end = d.xpos[part].copy()

    ejected = end[1] < FRONT_Y                  # pushed past the bed front edge
    dropped = end[2] < BED_TOP_Z - 0.03         # fell off the bed (into the bin)

    problems = []
    if not on_bed:
        problems.append(f"part not on bed at start (pos {np.round(start,3)})")
    if not ejected:
        problems.append(f"part not pushed past front edge (y={end[1]*1000:.0f}mm, edge {FRONT_Y*1000:.0f})")
    if not dropped:
        problems.append(f"part did not drop off the bed (z={end[2]*1000:.0f}mm)")

    print(f"start {np.round(start,3).tolist()} (on bed={on_bed})")
    print(f"end   {np.round(end,3).tolist()}  ejected={ejected} dropped={dropped}")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: cool -> open door -> sweep -> part ejected into the bin")
    return 0


if __name__ == "__main__":
    sys.exit(main())
