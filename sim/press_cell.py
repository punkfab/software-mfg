"""press_cell.py — a self-reacting C-frame press end-effector (sim).

Pressing needs a reaction UNDER the part; the arm can't provide it. So the tool is
a C-frame: a powered ram on top, an anvil on the bottom arm reaching under the
workpiece. The press force loops inside the C-frame — the arm/mount never sees it.
That self-reaction is the whole point, and this sim measures it.

Model (chosen for robustness — body↔body press contact chatters badly at these
force/mass scales): the bearing rides the RAM into the bracket bore; the press-fit
resistance is a spring on the ram's slide (rises with depth → FIT at the seat);
the ram is FORCE-driven (a motor = the toggle/lead-screw giving high force). The
whole tool hangs from a MOUNT slide (the arm); its hold force stays ~gravity iff
the frame is self-reacting.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np


def _cal(name, default):
    """Read a calibrated value from the tracked parameter vector, falling back to
    the literal if the store is missing. The sim consumes reality's numbers, not
    hardcoded guesses — and the staleness stamp (calibration/) says how far to
    trust them. Fallback keeps the sim runnable before anything is anchored."""
    try:
        root = str(Path(__file__).resolve().parent.parent)
        if root not in sys.path:
            sys.path.insert(0, root)
        from calibration.store import CalibrationStore
        store = CalibrationStore.load(Path(root) / "calibration" / "store.json")
        return store.value(name, default)
    except Exception:
        return default


SEAT_DEPTH = _cal("seat_depth", 0.008)    # bearing travel into the bore to seat (calibrated)
PRESS_FORCE = _cal("press_force", 300.0)  # press-fit resistance at the seat, N (calibrated)
FIT_STIFFNESS = PRESS_FORCE / SEAT_DEPTH  # modelled as ram-slide stiffness (robust)
RAM_DRIVE = 360.0                         # force the ram delivers (> fit -> it seats)
_G = mujoco.mjtGeom


def build_spec():
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.compiler.autolimits = True
    wb = spec.worldbody
    wb.add_light(pos=[0.1, -0.2, 0.4], dir=[-0.1, 0.2, -1])
    floor = wb.add_geom()
    floor.name, floor.type, floor.size, floor.rgba = "floor", _G.mjGEOM_PLANE, [0, 0, 0.05], [0.3, 0.32, 0.36, 1]
    floor.contype, floor.conaffinity = 0, 0   # visual: tool hangs from the mount

    # --- press tool hangs from a MOUNT slide (the arm holding it) ---
    tool = wb.add_body()
    tool.name, tool.pos, tool.mass = "cframe", [0, 0, 0.0], 1.2
    mj = tool.add_joint()
    mj.name, mj.type, mj.axis, mj.range = "mount_z", mujoco.mjtJoint.mjJNT_SLIDE, [0, 0, 1], [-0.05, 0.05]
    mj.armature = 1.0

    # C-frame: back column + top arm (ram guide) + bottom arm (anvil) + bracket/bore
    for name, size, pos in [
        ("col", [0.012, 0.02, 0.06], [-0.05, 0, 0.06]),
        ("top_arm", [0.05, 0.02, 0.012], [0, 0, 0.116]),
        ("anvil", [0.05, 0.03, 0.012], [0, 0, 0.012]),
        ("bracket", [0.03, 0.025, 0.012], [0, 0, 0.036]),
    ]:
        g = tool.add_geom()
        g.name, g.type, g.size, g.pos = name, _G.mjGEOM_BOX, size, pos
        g.rgba = [0.3, 0.34, 0.4, 1] if name in ("col", "top_arm") else [0.45, 0.5, 0.55, 1]
    bore = tool.add_geom()
    bore.name, bore.type, bore.size, bore.pos = "bore", _G.mjGEOM_CYLINDER, [0.0062, 0.012, 0], [0, 0, 0.036]
    bore.rgba, bore.contype, bore.conaffinity = [0.1, 0.1, 0.12, 1], 0, 0

    # --- ram (carries the bearing); slide with the press-fit spring; force-driven ---
    ram = tool.add_body()
    ram.name, ram.pos, ram.mass = "ram", [0, 0, 0.044], 0.3
    rj = ram.add_joint()
    rj.name, rj.type, rj.axis = "ram_z", mujoco.mjtJoint.mjJNT_SLIDE, [0, 0, 1]
    rj.range = [-SEAT_DEPTH, 0.05]                 # lower limit = the seat shoulder
    rj.limited = mujoco.mjtLimited.mjLIMITED_TRUE
    rj.stiffness = [FIT_STIFFNESS, 0, 0]           # press-fit resistance (rises with depth)
    rj.damping = [400.0, 0, 0]                     # near-critical -> smooth, no overshoot
    rj.armature = 1.0
    bearing = ram.add_geom()                       # the bearing, at the bore mouth
    bearing.name, bearing.type, bearing.size, bearing.pos = "bearing", _G.mjGEOM_CYLINDER, [0.006, 0.004, 0], [0, 0, 0]
    bearing.rgba, bearing.contype, bearing.conaffinity = [0.85, 0.55, 0.1, 1], 0, 0
    shaft = ram.add_geom()
    shaft.name, shaft.type, shaft.size, shaft.pos = "ram_shaft", _G.mjGEOM_CYLINDER, [0.005, 0.03, 0], [0, 0, 0.034]
    shaft.rgba, shaft.contype, shaft.conaffinity = [0.6, 0.62, 0.66, 1], 0, 0

    # actuators
    ma = spec.add_actuator()           # the arm holding the tool (stiff position hold)
    ma.name, ma.target, ma.trntype = "mount_z", "mount_z", mujoco.mjtTrn.mjTRN_JOINT
    ma.set_to_position(kp=8000, kv=200)
    ma.forcerange = [-800, 800]
    ra = spec.add_actuator()           # the ram drive: a FORCE motor (toggle/lead-screw)
    ra.name, ra.target, ra.trntype = "ram_z", "ram_z", mujoco.mjtTrn.mjTRN_JOINT
    ra.set_to_motor()                  # force = ctrl (the toggle/lead-screw drive)
    ra.forcerange = [-600, 600]
    return spec


def build_model():
    return build_spec().compile()


# (label, ram drive force [N, +up/-down], seconds)
PRESS_SEQUENCE = [
    ("settle", 0.0, 0.3),
    ("press", -RAM_DRIVE, 1.6),     # drive the bearing down into the bore to seat
    ("dwell", -RAM_DRIVE, 0.5),
    ("retract", 250.0, 0.8),        # pull the ram back up
]


def run_press(m, d, on_frame=None, fps=30, record=None):
    ram_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "ram_z")
    mnt_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "mount_z")
    ram_q = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "ram_z")]
    mujoco.mj_forward(m, d)
    spf = max(1, round((1.0 / fps) / m.opt.timestep))
    k = 0
    for label, force, secs in PRESS_SEQUENCE:
        for _ in range(max(1, int(secs / m.opt.timestep))):
            d.ctrl[ram_ci] = force
            d.ctrl[mnt_ci] = 0.0
            mujoco.mj_step(m, d)
            if record is not None and label in ("press", "dwell"):
                insertion = -float(d.qpos[ram_q])
                record["press_force"] = max(record.get("press_force", 0.0), FIT_STIFFNESS * max(insertion, 0))
                record["mount_force"] = max(record.get("mount_force", 0.0), abs(d.actuator_force[mnt_ci]))
                if label == "dwell":
                    record["insertion"] = insertion
            if on_frame is not None and k % spf == 0:
                on_frame()
            k += 1


if __name__ == "__main__":
    m = build_model()
    print(f"press_cell: nbody={m.nbody} nu={m.nu}  fit={PRESS_FORCE:.0f}N seat={SEAT_DEPTH*1000:.0f}mm")
