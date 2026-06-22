#!/usr/bin/env python3
"""Gate: the bender's real wire lands in the arm's workcell under the shear.

Closes the geometric loop check — the produced staple becomes capsule segments in
the tool-changer scene, positioned where the presented blade closes.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from toolchanger import DATUM_POS, build_model  # noqa: E402
from wirebender_cell import staple_in_workcell  # noqa: E402

BLADE_X = DATUM_POS[0] + 0.03   # where the presented shear blade closes


def main() -> int:
    wire = np.array(staple_in_workcell([BLADE_X, DATUM_POS[1]], 0.086))
    m = build_model(wire_points=wire)
    nseg = sum(1 for i in range(m.ngeom)
               if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith("wire"))

    problems = []
    if nseg != len(wire) - 1:
        problems.append(f"expected {len(wire)-1} wire segments, model has {nseg}")
    if not (wire[:, 0].min() <= BLADE_X <= wire[:, 0].max()):
        problems.append("blade-close X is not over the wire")

    print(f"real wire in workcell: {nseg} segments, "
          f"x[{wire[:,0].min():.3f},{wire[:,0].max():.3f}] m, blade-close x={BLADE_X:.3f}")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: bender's real wire placed in the arm's workcell under the shear")
    return 0


if __name__ == "__main__":
    sys.exit(main())
