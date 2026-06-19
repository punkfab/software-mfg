#!/usr/bin/env python3
"""Validate the vendored SO-101 MuJoCo model — Phase 1 gate.

Compiles scene.xml, asserts the expected 6-DOF structure, then runs a scripted
joint move under the model's position actuators and confirms (a) the joints track
their commanded targets and (b) the end effector actually moves. This proves the
model loads, forward kinematics work, and the control interface (ctrl = desired
joint angles) is wired correctly.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "so101" / "scene.xml"
EXPECTED_JOINTS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper",
]


def main() -> int:
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)

    problems: list[str] = []
    if m.nu != 6:
        problems.append(f"expected 6 actuators, got {m.nu}")
    joints = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(m.njnt)]
    if joints != EXPECTED_JOINTS:
        problems.append(f"unexpected joints: {joints}")

    tip = m.nbody - 1  # moving_jaw — the end effector body
    tip_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, tip)

    mujoco.mj_forward(m, d)
    home = d.xpos[tip].copy()

    # Scripted move: position actuators take desired joint angles in ctrl.
    target = np.zeros(m.nu)
    target[0], target[1] = 1.0, -0.8  # shoulder_pan, shoulder_lift
    d.ctrl[:] = target
    for _ in range(2000):
        mujoco.mj_step(m, d)

    moved = d.xpos[tip].copy()
    disp = float(np.linalg.norm(moved - home))
    track_err = float(np.abs(d.qpos[:2] - target[:2]).max())

    if disp < 0.05:
        problems.append(f"end effector barely moved ({disp:.3f} m) — actuation not working")
    if track_err > 0.2:
        problems.append(f"joints did not track targets (err {track_err:.3f} rad)")

    print(f"model: nq={m.nq} nv={m.nv} nu={m.nu} nbody={m.nbody} nmesh={m.nmesh} "
          f"mass={sum(m.body_mass):.3f} kg")
    print(f"actuators: position control (ctrl = desired joint angles, rad)")
    print(f"end effector body: {tip_name}")
    print(f"  home  {np.round(home, 3)}")
    print(f"  moved {np.round(moved, 3)}  (Δ={disp:.3f} m, tracking err={track_err:.3f} rad)")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: SO-101 compiles, position control tracks, FK moves the end effector")
    return 0


if __name__ == "__main__":
    sys.exit(main())
