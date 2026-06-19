"""printer_cell.py — Bambu P1S as a cell, with eject-in-place part removal.

A standalone MuJoCo scene (no arm needed — eject-in-place uses the printer's own
mechanism): the enclosed bed, a cooled printed part, an auto-opening front door,
and a full-width sweep blade (CAD: parts/ejector_blade.py) that pushes the part
off the front edge, out the door, into a catch bin.

This is the printer cell's *physical* side. Its control side (LAN/MQTT: start
print, read bed-temp/done) is a separate adapter — see PLAN.md (printer cell).

    MUJOCO_GL=osmesa python sim/printer_cell.py   # preview still
"""

import mujoco
import numpy as np

BED = 0.256
BED_TOP_Z = 0.10
FRONT_Y = -BED / 2                 # bed front edge (part exits past this, -y)
PART_HALF = np.array([0.02, 0.02, 0.015])
PART_START = np.array([0.0, 0.02, BED_TOP_Z + PART_HALF[2] + 0.001])
SWEEP_BACK, SWEEP_FRONT = 0.14, -0.185   # blade Y travel (back -> past front edge)
_G = mujoco.mjtGeom


def _panel(body, name, size, pos, rgba, quat=(1, 0, 0, 0)):
    g = body.add_geom()
    g.name, g.type, g.size, g.pos, g.rgba, g.quat = name, _G.mjGEOM_BOX, size, pos, rgba, quat
    g.contype, g.conaffinity = 0, 0     # enclosure panels are visual only
    return g


def build_spec():
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    wb = spec.worldbody
    wb.add_light(pos=[0.1, -0.2, 0.5], dir=[-0.1, 0.2, -1])
    floor = wb.add_geom()
    floor.name, floor.type, floor.size = "floor", _G.mjGEOM_PLANE, [0, 0, 0.05]
    floor.rgba = [0.3, 0.32, 0.36, 1]

    # --- printer body: stand + heated bed plate (the part rests on the bed) ---
    stand = wb.add_geom()
    stand.name, stand.type = "stand", _G.mjGEOM_BOX
    stand.size = [BED / 2, BED / 2, (BED_TOP_Z - 0.01) / 2]
    stand.pos = [0, 0, (BED_TOP_Z - 0.01) / 2]
    stand.rgba = [0.2, 0.2, 0.23, 1]
    bed = wb.add_geom()
    bed.name, bed.type = "bed", _G.mjGEOM_BOX
    bed.size = [BED / 2, BED / 2, 0.005]
    bed.pos = [0, 0, BED_TOP_Z - 0.005]
    bed.rgba = [0.12, 0.12, 0.14, 1]

    # visual enclosure (back + sides + top frame)
    _panel(wb, "wall_back", [BED / 2, 0.004, 0.13], [0, BED / 2, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.25])
    _panel(wb, "wall_left", [0.004, BED / 2, 0.13], [-BED / 2, 0, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.25])
    _panel(wb, "wall_right", [0.004, BED / 2, 0.13], [BED / 2, 0, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.25])
    _panel(wb, "top", [BED / 2, BED / 2, 0.004], [0, 0, BED_TOP_Z + 0.26], [0.4, 0.45, 0.5, 0.25])

    # --- auto-opening front door (vertical hinge at front-left). Visual ---
    door = wb.add_body()
    door.name = "door"
    door.pos = [-BED / 2, FRONT_Y, BED_TOP_Z + 0.13]
    dj = door.add_joint()
    dj.name, dj.type, dj.axis, dj.range = "door", mujoco.mjtJoint.mjJNT_HINGE, [0, 0, 1], [0, 1.9]
    _panel(door, "door_panel", [BED / 2, 0.004, 0.13], [BED / 2, 0, 0], [0.5, 0.7, 0.9, 0.3])

    # --- sweep ejector: blade on a Y slide, just above the bed surface ---
    sweep = wb.add_body()
    sweep.name = "sweep"
    sweep.pos = [0, 0, BED_TOP_Z + 0.031]
    sweep.mass = 0.25
    sj = sweep.add_joint()
    sj.name, sj.type, sj.axis = "sweep", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0]
    sj.range = [SWEEP_FRONT, SWEEP_BACK]
    blade = sweep.add_geom()
    blade.name, blade.type = "blade", _G.mjGEOM_BOX
    blade.size = [BED / 2, 0.004, 0.029]
    blade.rgba = [0.85, 0.3, 0.2, 1]

    # --- the cooled printed part (free body resting on the bed) ---
    part = wb.add_body()
    part.name = "part"
    part.pos = PART_START.tolist()
    pj = part.add_freejoint()
    pj.name = "part_free"
    pg = part.add_geom()
    pg.name, pg.type, pg.size = "part_geom", _G.mjGEOM_BOX, PART_HALF.tolist()
    pg.rgba = [0.9, 0.75, 0.2, 1]

    # --- catch bin in front of the door (collidable) ---
    for nm, size, pos in [
        ("bin_floor", [0.09, 0.07, 0.004], [0, FRONT_Y - 0.13, 0.004]),
        ("bin_front", [0.09, 0.004, 0.03], [0, FRONT_Y - 0.20, 0.03]),
        ("bin_l", [0.004, 0.07, 0.03], [-0.09, FRONT_Y - 0.13, 0.03]),
        ("bin_r", [0.004, 0.07, 0.03], [0.09, FRONT_Y - 0.13, 0.03]),
    ]:
        g = wb.add_geom()
        g.name, g.type, g.size, g.pos = nm, _G.mjGEOM_BOX, size, pos
        g.rgba = [0.25, 0.5, 0.3, 1]

    # actuators
    da = spec.add_actuator()
    da.name, da.target, da.trntype = "door", "door", mujoco.mjtTrn.mjTRN_JOINT
    da.set_to_position(kp=6.0, kv=1.0)
    da.ctrlrange = [0, 1.9]
    sa = spec.add_actuator()
    sa.name, sa.target, sa.trntype = "sweep", "sweep", mujoco.mjtTrn.mjTRN_JOINT
    sa.set_to_position(kp=400.0, kv=40.0)
    sa.ctrlrange = [SWEEP_FRONT, SWEEP_BACK]
    sa.forcerange = [-60, 60]

    return spec


def build_model():
    return build_spec().compile()


# Eject sequence: (label, door_ctrl | None, sweep_ctrl | None, seconds)
EJECT_SEQUENCE = [
    ("cool / dwell", None, None, 0.5),
    ("open door", 1.9, None, 1.0),
    ("sweep eject", None, SWEEP_FRONT, 2.4),
    ("sweep return", None, SWEEP_BACK, 1.0),
    ("close door", 0.0, None, 0.8),
]


def run_eject(m, d, on_frame=None, fps=30):
    """Run the eject sequence, ramping the door + sweep position actuators."""
    door_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "door")
    sweep_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "sweep")
    sweep_q = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "sweep")]
    d.qpos[sweep_q] = SWEEP_BACK
    mujoco.mj_forward(m, d)

    cur = np.zeros(m.nu)
    cur[sweep_ci] = SWEEP_BACK
    d.ctrl[:] = cur
    spf = max(1, round((1.0 / fps) / m.opt.timestep))
    k = 0
    for _label, door, sweep, secs in EJECT_SEQUENCE:
        nxt = cur.copy()
        if door is not None:
            nxt[door_ci] = door
        if sweep is not None:
            nxt[sweep_ci] = sweep
        nsteps = max(1, int(secs / m.opt.timestep))
        for i in range(nsteps):
            d.ctrl[:] = cur + (nxt - cur) * ((i + 1) / nsteps)
            mujoco.mj_step(m, d)
            if on_frame is not None and (k % spf) == 0:
                on_frame()
            k += 1
        cur = nxt


if __name__ == "__main__":
    import subprocess
    from pathlib import Path

    m = build_model()
    d = mujoco.MjData(m)
    # start the blade at the back, door closed
    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "sweep")]] = SWEEP_BACK
    mujoco.mj_forward(m, d)
    out = Path(__file__).resolve().parent.parent / "exports" / "renders"
    out.mkdir(parents=True, exist_ok=True)
    r = mujoco.Renderer(m, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, -0.05, 0.12]
    cam.distance, cam.azimuth, cam.elevation = 0.9, 120, -22
    r.update_scene(d, camera=cam)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", "640x480", "-i", "-", "-frames:v", "1", str(out / "printer_cell.png")],
                   input=r.render().tobytes(), check=True)
    print(f"printer_cell: nbody={m.nbody} nu={m.nu}  bed {BED*1000:.0f}mm  -> {out/'printer_cell.png'}")
