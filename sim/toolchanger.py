"""toolchanger.py — extend the workcell with a tool rack + self-actuated shear.

Builds on workcell.build_spec() (which already composes the SO-101 from the
outside) and adds, all via MjSpec so the vendored snapshot stays untouched:

  * a tool RACK (static cradle) holding a parked shear tool,
  * a SHEAR TOOL as a free body with its own actuated blade — the tool supplies
    its own process force; the arm only positions it (CONCEPT.md principle #2),
  * a WELD equality between the arm's end body and the tool, toggled at runtime
    via data.eq_active — the abstraction of the electropermanent magnet (an
    attach/detach event, not full magnetics; CONCEPT.md §5.2),
  * a thin WIRE proxy at the datum for the shear to close on.

The weld relpose hangs the tool TOOL_HANG below the end-effector frame, so the
demo aims the arm TOOL_HANG above the parked tool and couples with ~zero snap.
"""

from pathlib import Path
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from workcell import DATUM_POS, build_spec  # noqa: E402

END_BODY = "moving_jaw_so101_v1"          # arm end-effector body the tool welds to
TOOL_HANG = 0.05                           # tool origin sits this far below the end frame
RACK_XY = np.array([0.03, 0.20])           # tool rack location (left, clear of the table)
TOOL_BIT = 4                               # contype/conaffinity: tool ↔ cradle only
TOOL_PARK_Z = 0.11                         # parked tool-origin height (rests on the rack)
MATE_TARGET = np.array([RACK_XY[0], RACK_XY[1], TOOL_PARK_Z + TOOL_HANG])
WIRE_POS = DATUM_POS + np.array([0.0, 0.0, 0.005])

_C = mujoco.mjtGeom


def build_spec():  # noqa: F811 - wraps workcell.build_spec and extends it
    from workcell import build_spec as _workcell_spec
    spec = _workcell_spec()
    wb = spec.worldbody

    # --- tool rack: a static cradle the tool's housing bottom rests on ---
    HOUSING_BOTTOM = TOOL_PARK_Z - 0.028
    post = wb.add_geom()
    post.name = "rack_post"
    post.type = _C.mjGEOM_BOX
    post.size = [0.022, 0.022, HOUSING_BOTTOM / 2]
    post.pos = [RACK_XY[0], RACK_XY[1], HOUSING_BOTTOM / 2]
    post.rgba = [0.30, 0.30, 0.34, 1.0]
    post.contype, post.conaffinity = TOOL_BIT, TOOL_BIT   # only the tool rests here

    # --- shear tool: free body, parked above the rack ---
    tool = wb.add_body()
    tool.name = "shear_tool"
    tool.pos = [RACK_XY[0], RACK_XY[1], TOOL_PARK_Z]
    tool.mass = 0.08
    fj = tool.add_freejoint()
    fj.name = "tool_free"

    housing = tool.add_geom()
    housing.name = "tool_housing"
    housing.type = _C.mjGEOM_BOX
    housing.size = [0.016, 0.016, 0.028]
    housing.rgba = [0.15, 0.55, 0.75, 1.0]
    housing.contype, housing.conaffinity = TOOL_BIT, TOOL_BIT  # contacts only the cradle

    # coupling plate on top (the kinematic-coupling face mates here). Visual only.
    plate = tool.add_geom()
    plate.name = "tool_coupling"
    plate.type = _C.mjGEOM_CYLINDER
    plate.size = [0.025, 0.004, 0]
    plate.pos = [0, 0, 0.032]
    plate.rgba = [0.85, 0.85, 0.2, 1.0]
    plate.contype, plate.conaffinity = 0, 0

    # fixed lower jaw of the shear, reaching forward (+x). Kept above the housing
    # bottom so nothing hangs below the rack-contact plane (parking stability).
    lower = tool.add_geom()
    lower.name = "shear_lower"
    lower.type = _C.mjGEOM_BOX
    lower.size = [0.018, 0.003, 0.0015]
    lower.pos = [0.030, 0, -0.020]
    lower.rgba = [0.7, 0.7, 0.72, 1.0]
    lower.contype, lower.conaffinity = 0, 0    # visual; real severing is Phase 3

    # --- moving blade: hinged child body with its own actuator ---
    blade = tool.add_body()
    blade.name = "shear_blade"
    blade.pos = [0.014, 0, -0.020]
    blade.mass = 0.01
    bj = blade.add_joint()
    bj.name = "shear"
    bj.type = mujoco.mjtJoint.mjJNT_HINGE
    bj.axis = [0, 1, 0]
    bj.range = [0.0, 0.9]            # 0 = open, 0.9 rad = closed/cut
    bj.armature = 0.002             # rotor inertia — stabilises the light blade
    bj.damping = [0.05, 0, 0]
    bg = blade.add_geom()
    bg.name = "blade_edge"
    bg.type = _C.mjGEOM_BOX
    bg.size = [0.018, 0.0025, 0.0015]
    bg.pos = [0.016, 0, 0]
    bg.rgba = [0.85, 0.2, 0.2, 1.0]
    bg.contype, bg.conaffinity = 0, 0          # visual blade; no internal contact

    act = spec.add_actuator()
    act.name = "shear"
    act.target = "shear"
    act.trntype = mujoco.mjtTrn.mjTRN_JOINT
    act.set_to_position(kp=12.0, kv=1.0)
    act.ctrlrange = [0.0, 0.9]
    act.forcerange = [-15.0, 15.0]

    # --- wire proxy at the datum (what the shear closes on) ---
    wire = wb.add_geom()
    wire.name = "wire"
    wire.type = _C.mjGEOM_CYLINDER
    wire.size = [0.0008, 0.03, 0]       # thin rod along its local z
    wire.pos = WIRE_POS.tolist()
    wire.quat = [0.7071, 0.7071, 0, 0]  # lay it along the y axis
    wire.rgba = [0.6, 0.6, 0.62, 1.0]
    wire.contype, wire.conaffinity = 0, 0

    # --- weld: EPM stand-in. Inactive; relpose hangs tool below the end frame ---
    dock = spec.add_equality()
    dock.name = "dock"
    dock.type = mujoco.mjtEq.mjEQ_WELD
    dock.objtype = mujoco.mjtObj.mjOBJ_BODY
    dock.name1 = END_BODY
    dock.name2 = "shear_tool"
    dock.active = False
    # eq_data = anchor(3) + relpose pos(3) + relpose quat(4) + torquescale(1)
    dock.data = [0, 0, 0,  0, 0, -TOOL_HANG,  1, 0, 0, 0,  1]

    return spec


def build_model():
    return build_spec().compile()


if __name__ == "__main__":
    m = build_model()
    print(f"toolchanger: nbody={m.nbody} nu={m.nu} neq={m.neq} nq={m.nq} nv={m.nv}")
    print(f"  actuators: {[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]}")
    print(f"  MATE_TARGET={MATE_TARGET.tolist()}  DATUM={DATUM_POS.tolist()}")
