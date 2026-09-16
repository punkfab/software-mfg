"""ik.py — minimal damped-least-squares inverse kinematics for the SO-101.

Positional IK only (3-DOF target) over the arm joints (gripper excluded). Good enough to drive
Cartesian placement in the workcell; swap for a richer solver if orientation control is later needed.

FIX (base-freejoint safe): the arm joints are located by their actual qpos/dof addresses, not assumed
to be qpos[:5]. On the LeKiwi the base free joint occupies qpos[0:7] / dof[0:6], so the old
`data.qpos[:5]` indexed the base pose, not the arm — the arm never moved. Pass `joint_names=` (the arm
joints, in order) whenever a base joint is present; omit it for a fixed-base arm (legacy behaviour).
"""

import mujoco
import numpy as np

ARM_DOFS = 5  # shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll


def arm_indices(model, joint_names=None, n=ARM_DOFS):
    """Return (qpos_adr, dof_adr, lo, hi) for the arm joints. With `joint_names`, resolve each by name
    (robust to a base freejoint offsetting the addresses); otherwise fall back to the first `n` joints.
    lo/hi are +-inf for UNLIMITED joints (an unlimited joint has jnt_range [0,0]; clipping to that would
    pin it at zero)."""
    if joint_names:
        jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn) for jn in joint_names]
        if any(j < 0 for j in jids):
            missing = [jn for jn, j in zip(joint_names, jids) if j < 0]
            raise ValueError(f"ik: joint(s) not found: {missing}")
        jids = np.array(jids)
    else:
        jids = np.arange(n)
    qadr = model.jnt_qposadr[jids]
    dadr = model.jnt_dofadr[jids]
    limited = model.jnt_limited[jids].astype(bool)
    lo = np.where(limited, model.jnt_range[jids, 0], -np.inf)
    hi = np.where(limited, model.jnt_range[jids, 1], np.inf)
    return qadr, dadr, lo, hi


def solve_ik(model, data, body_id, target, *, joint_names=None, iters=300, tol=5e-4, damping=1e-2):
    """Solve for arm joint angles placing `body_id`'s origin at `target` (world xyz).

    Mutates the arm's qpos entries in place and returns (qpos_arm, final_error_norm). Clamps to joint
    limits. Pass `joint_names` (arm joints in order) when a base joint offsets the qpos/dof addresses.
    """
    target = np.asarray(target, dtype=float)
    qadr, dadr, lo, hi = arm_indices(model, joint_names)
    jacp = np.zeros((3, model.nv))

    err_norm = np.inf
    for _ in range(iters):
        mujoco.mj_forward(model, data)
        err = target - data.xpos[body_id]
        err_norm = float(np.linalg.norm(err))
        if err_norm < tol:
            break
        mujoco.mj_jacBody(model, data, jacp, None, body_id)
        J = jacp[:, dadr]                                     # only the arm's dof columns
        dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), err)
        data.qpos[qadr] = np.clip(data.qpos[qadr] + dq, lo, hi)

    return data.qpos[qadr].copy(), err_norm
