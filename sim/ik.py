"""ik.py — minimal damped-least-squares inverse kinematics for the SO-101.

Positional IK only (3-DOF target) over the 5 arm joints (gripper excluded). Good
enough to drive Cartesian placement in the workcell; swap for a richer solver if
orientation control is later needed.
"""

import mujoco
import numpy as np

ARM_DOFS = 5  # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll


def solve_ik(model, data, body_id, target, *, iters=300, tol=5e-4, damping=1e-2):
    """Solve for arm joint angles placing `body_id`'s origin at `target` (world xyz).

    Mutates data.qpos[:ARM_DOFS] in place and returns it. Clamps to joint limits.
    Returns (qpos_arm, final_error_norm).
    """
    target = np.asarray(target, dtype=float)
    jacp = np.zeros((3, model.nv))
    lo, hi = model.jnt_range[:ARM_DOFS, 0], model.jnt_range[:ARM_DOFS, 1]

    err_norm = np.inf
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[body_id]
        err_norm = float(np.linalg.norm(err))
        if err_norm < tol:
            break
        mujoco.mj_jacBody(model, data, jacp, None, body_id)
        J = jacp[:, :ARM_DOFS]
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), err)
        data.qpos[:ARM_DOFS] = np.clip(data.qpos[:ARM_DOFS] + dq, lo, hi)

    return data.qpos[:ARM_DOFS].copy(), err_norm
