#!/usr/bin/env python3
"""Phase 2 gate: the tool-change mechanism actually works.

Runs the same segment sequence as the demo (no rendering) and asserts the tool
is parked, coupled, carried (lifted off the rack, tracking the end frame at the
weld offset), presented at the datum, sheared (blade closes), and returned +
decoupled onto the rack.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import ik  # noqa: E402
from toolchanger import DATUM_POS, RACK_XY, TOOL_HANG, build_model  # noqa: E402
from toolchange_demo import SEGMENTS  # noqa: E402


def run():
    m = build_model()
    d, scratch = mujoco.MjData(m), mujoco.MjData(m)
    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    tool_b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shear_tool")
    dock = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "dock")
    shear_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "shear")
    blade_q = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "shear")]
    mujoco.mj_forward(m, d)

    def weld_in_place():
        ipos, iquat, rpos, rquat = (np.zeros(3), np.zeros(4), np.zeros(3), np.zeros(4))
        mujoco.mju_negPose(ipos, iquat, d.xpos[tip], d.xquat[tip])
        mujoco.mju_mulPose(rpos, rquat, ipos, iquat, d.xpos[tool_b], d.xquat[tool_b])
        m.eq_data[dock, 3:6], m.eq_data[dock, 6:10], m.eq_data[dock, 10] = rpos, rquat, 1

    log, max_blade = {}, 0.0
    cur, prev_weld = np.zeros(m.nu), False
    for label, target, shear, weld, secs in SEGMENTS:
        nxt = cur.copy()
        if target is not None:
            scratch.qpos[:] = d.qpos
            nxt[: ik.ARM_DOFS], _ = ik.solve_ik(m, scratch, tip, np.asarray(target))
        nxt[shear_ci] = shear
        if weld and not prev_weld:
            weld_in_place()
        d.eq_active[dock] = 1 if weld else 0
        prev_weld = weld
        nsteps = max(1, int(secs / m.opt.timestep))
        for i in range(nsteps):
            d.ctrl[:] = cur + (nxt - cur) * ((i + 1) / nsteps)
            mujoco.mj_step(m, d)
            max_blade = max(max_blade, float(d.qpos[blade_q]))
        cur = nxt
        log[label] = dict(tool=d.xpos[tool_b].copy(), jaw=d.xpos[tip].copy())
    log["_max_blade"] = max_blade
    return log


def main() -> int:
    log = run()
    park_z = log["approach rack"]["tool"][2]
    lift = log["lift tool"]
    present = log["present at datum"]
    cut_angle = log["_max_blade"]
    end = log["retract"]

    problems = []
    if lift["tool"][2] - park_z < 0.02:
        problems.append(f"tool not lifted off rack (Δz={1000*(lift['tool'][2]-park_z):.0f} mm)")
    carry_gap = np.linalg.norm(lift["tool"] - lift["jaw"])
    if abs(carry_gap - TOOL_HANG) > 0.02:
        problems.append(f"tool not tracking end frame while carried (gap {carry_gap*1000:.0f} mm)")
    if np.linalg.norm(present["tool"][:2] - DATUM_POS[:2]) > 0.05:
        problems.append("tool not presented at the datum")
    if cut_angle < 0.8:
        problems.append(f"shear did not close (blade {cut_angle:.2f} rad)")
    if np.linalg.norm(end["tool"][:2] - RACK_XY) > 0.05:
        problems.append("tool not returned to the rack")

    print(f"park tool z={park_z*1000:.0f}mm  lift z={lift['tool'][2]*1000:.0f}mm  "
          f"carry gap={carry_gap*1000:.0f}mm (hang={TOOL_HANG*1000:.0f})")
    print(f"present xy err={np.linalg.norm(present['tool'][:2]-DATUM_POS[:2])*1000:.0f}mm  "
          f"shear close={cut_angle:.2f}rad  return xy err={np.linalg.norm(end['tool'][:2]-RACK_XY)*1000:.0f}mm")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: park -> couple -> carry -> present -> shear -> return -> decouple")
    return 0


if __name__ == "__main__":
    sys.exit(main())
