#!/usr/bin/env python3
"""Phase 1 gate: workcell compiles and IK reaches the work datum.

Builds the MjSpec workcell, solves positional IK so the end effector reaches the
fiducial datum, then verifies under position control that the arm actually drives
there. Proves the compose-from-outside workcell + the IK helper both work.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import ik  # noqa: E402
from workcell import DATUM_POS, build_model  # noqa: E402


def main() -> int:
    m = build_model()
    d = mujoco.MjData(m)
    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")

    # Aim at an approach waypoint clear above the datum fixture (free space).
    target = DATUM_POS + np.array([0.0, 0.0, 0.06])
    q_arm, ik_err = ik.solve_ik(m, d, tip, target)

    # Drive there under position control and confirm the tip arrives.
    d2 = mujoco.MjData(m)
    d2.ctrl[:ik.ARM_DOFS] = q_arm
    for _ in range(2000):
        mujoco.mj_step(m, d2)
    reached = float(np.linalg.norm(d2.xpos[tip] - target))

    problems = []
    if ik_err > 5e-3:
        problems.append(f"IK did not converge (residual {ik_err*1000:.1f} mm)")
    if reached > 0.02:
        problems.append(f"arm did not reach datum under control ({reached*1000:.1f} mm off)")

    print(f"workcell: nbody={m.nbody} ngeom={m.ngeom}  datum={np.round(DATUM_POS,3).tolist()}")
    print(f"IK residual: {ik_err*1000:.2f} mm   controlled reach error: {reached*1000:.2f} mm")
    print(f"arm solution (rad): {np.round(q_arm,3).tolist()}")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: workcell compiles, IK converges, arm reaches the work datum")
    return 0


if __name__ == "__main__":
    sys.exit(main())
