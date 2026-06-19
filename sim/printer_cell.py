"""printer_cell.py — Bambu P1S as a cell, eject-in-place via the toolhead.

Kinematically faithful to the P1S (CoreXY): the TOOLHEAD moves in X-Y at a fixed
gantry height (never in Z); the BED moves in Z. Eject-in-place therefore uses the
printer's own motion — no added sweep hardware, the standard print-farm trick:

  cool -> open door -> drive the toolhead into the cooled part and knock it off
  the front edge into a bin -> bed drops (post-cycle) -> close door.

This is the printer cell's *physical* side. Its control side (LAN/MQTT: start a
sliced 3MF, read bed-temp / progress / done) is a separate adapter — see PLAN.md.

    MUJOCO_GL=osmesa python sim/printer_cell.py   # preview still
"""

import mujoco
import numpy as np

BED = 0.256
BED_TOP_Z = 0.10                 # bed surface at bed_z = 0 (print height)
FRONT_Y = -BED / 2               # bed front edge; part exits past this (-y)
PART_HALF = np.array([0.02, 0.02, 0.015])
PART_START = np.array([0.0, -0.01, BED_TOP_Z + PART_HALF[2] + 0.001])
TH_PARK_Y = 0.12                 # toolhead parked behind the part (+y)
_G = mujoco.mjtGeom


def _vis(body, name, size, pos, rgba, quat=(1, 0, 0, 0)):
    g = body.add_geom()
    g.name, g.type, g.size, g.pos, g.rgba, g.quat = name, _G.mjGEOM_BOX, size, pos, rgba, quat
    g.contype, g.conaffinity = 0, 0          # visual only
    return g


def build_spec():
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    wb = spec.worldbody
    wb.add_light(pos=[0.1, -0.2, 0.5], dir=[-0.1, 0.2, -1])
    floor = wb.add_geom()
    floor.name, floor.type, floor.size = "floor", _G.mjGEOM_PLANE, [0, 0, 0.05]
    floor.rgba = [0.3, 0.32, 0.36, 1]

    # fixed frame posts (visual) + a visual enclosure
    for nm, x in [("post_l", -BED / 2), ("post_r", BED / 2)]:
        _vis(wb, nm, [0.006, 0.006, 0.18], [x, BED / 2, BED_TOP_Z + 0.18], [0.35, 0.37, 0.4, 1])
    _vis(wb, "wall_back", [BED / 2, 0.004, 0.13], [0, BED / 2, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.22])
    _vis(wb, "wall_left", [0.004, BED / 2, 0.13], [-BED / 2, 0, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.22])
    _vis(wb, "wall_right", [0.004, BED / 2, 0.13], [BED / 2, 0, BED_TOP_Z + 0.13], [0.5, 0.6, 0.7, 0.22])
    _vis(wb, "top", [BED / 2, BED / 2, 0.004], [0, 0, BED_TOP_Z + 0.26], [0.4, 0.45, 0.5, 0.2])

    # --- bed: moves in Z (print height at bed_z = 0; drops post-cycle) ---
    bed = wb.add_body()
    bed.name, bed.pos, bed.mass = "bed", [0, 0, BED_TOP_Z], 0.6
    bj = bed.add_joint()
    bj.name, bj.type, bj.axis, bj.range = "bed_z", mujoco.mjtJoint.mjJNT_SLIDE, [0, 0, 1], [-0.12, 0.0]
    bg = bed.add_geom()
    bg.name, bg.type, bg.size, bg.pos = "bed_plate", _G.mjGEOM_BOX, [BED / 2, BED / 2, 0.02], [0, 0, -0.02]
    bg.rgba = [0.12, 0.12, 0.14, 1]
    bg.solref, bg.solimp = [0.006, 1.0], [0.95, 0.99, 0.001, 0.5, 2]   # stiff contact (no sink-through)

    # --- toolhead: CoreXY, X-Y slides at fixed gantry height (never Z) ---
    th_x = wb.add_body()
    th_x.name, th_x.pos, th_x.mass = "th_x", [0, 0, BED_TOP_Z + 0.032], 0.3
    xj = th_x.add_joint()
    xj.name, xj.type, xj.axis, xj.range = "th_x", mujoco.mjtJoint.mjJNT_SLIDE, [1, 0, 0], [-BED / 2, BED / 2]
    _vis(th_x, "x_carriage", [0.022, 0.012, 0.012], [0, 0, 0], [0.4, 0.4, 0.45, 1])
    th = th_x.add_body()
    th.name, th.mass = "toolhead", 0.3
    yj = th.add_joint()
    yj.name, yj.type, yj.axis, yj.range = "th_y", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0], [-0.185, 0.14]
    # hotend block (the pusher) + nozzle cone; bottom ~2 mm above the bed surface
    hb = th.add_geom()
    hb.name, hb.type, hb.size, hb.pos = "hotend", _G.mjGEOM_BOX, [0.009, 0.009, 0.03], [0, 0, 0]
    hb.rgba = [0.2, 0.2, 0.22, 1]
    nz = th.add_geom()
    nz.name, nz.type, nz.size, nz.pos = "nozzle", _G.mjGEOM_CYLINDER, [0.003, 0.006, 0], [0, 0, -0.034]
    nz.rgba = [0.8, 0.7, 0.3, 1]
    nz.contype, nz.conaffinity = 0, 0

    # --- cooled printed part (free body on the bed) ---
    part = wb.add_body()
    part.name, part.pos = "part", PART_START.tolist()
    part.add_freejoint().name = "part_free"
    pg = part.add_geom()
    pg.name, pg.type, pg.size, pg.rgba = "part_geom", _G.mjGEOM_BOX, PART_HALF.tolist(), [0.9, 0.75, 0.2, 1]
    pg.solref, pg.solimp = [0.006, 1.0], [0.95, 0.99, 0.001, 0.5, 2]
    pg.friction = [1.0, 0.02, 0.001]

    # --- auto-opening front door (visual) ---
    door = wb.add_body()
    door.name, door.pos = "door", [-BED / 2, FRONT_Y, BED_TOP_Z + 0.13]
    dj = door.add_joint()
    dj.name, dj.type, dj.axis, dj.range = "door", mujoco.mjtJoint.mjJNT_HINGE, [0, 0, 1], [0, 1.9]
    _vis(door, "door_panel", [BED / 2, 0.004, 0.13], [BED / 2, 0, 0], [0.5, 0.7, 0.9, 0.3])

    # --- catch bin in front of the door (collidable) ---
    for nm, size, pos in [
        ("bin_floor", [0.09, 0.07, 0.004], [0, FRONT_Y - 0.13, 0.004]),
        ("bin_front", [0.09, 0.004, 0.03], [0, FRONT_Y - 0.20, 0.03]),
        ("bin_l", [0.004, 0.07, 0.03], [-0.09, FRONT_Y - 0.13, 0.03]),
        ("bin_r", [0.004, 0.07, 0.03], [0.09, FRONT_Y - 0.13, 0.03]),
    ]:
        g = wb.add_geom()
        g.name, g.type, g.size, g.pos, g.rgba = nm, _G.mjGEOM_BOX, size, pos, [0.25, 0.5, 0.3, 1]

    # actuators (position control)
    for name, kp, kv, frange, crange in [
        ("bed_z", 20000, 300, 200, [-0.12, 0.0]),   # stiff leadscrew (holds bed against gravity)
        ("th_x", 300, 30, 40, [-BED / 2, BED / 2]),
        ("th_y", 300, 30, 40, [-0.185, 0.14]),
        ("door", 6, 1, 10, [0, 1.9]),
    ]:
        a = spec.add_actuator()
        a.name, a.target, a.trntype = name, name, mujoco.mjtTrn.mjTRN_JOINT
        a.set_to_position(kp=kp, kv=kv)
        a.forcerange = [-frange, frange]
        a.ctrlrange = crange
    return spec


def build_model():
    return build_spec().compile()


# Eject sequence: (label, {actuator: target}, seconds). Toolhead knocks the part.
EJECT_SEQUENCE = [
    ("cool / dwell", {}, 0.5),
    ("open door", {"door": 1.9}, 1.0),
    ("toolhead behind part", {"th_x": 0.0, "th_y": TH_PARK_Y}, 0.8),
    ("knock part off front", {"th_y": -0.185}, 2.2),
    ("retract toolhead", {"th_y": TH_PARK_Y}, 0.8),
    ("bed drops (post-cycle)", {"bed_z": -0.10}, 0.9),
    ("close door", {"door": 0.0}, 0.8),
]
HOME = {"bed_z": 0.0, "th_x": 0.0, "th_y": TH_PARK_Y, "door": 0.0}


def run_eject(m, d, on_frame=None, fps=30):
    """Run the toolhead-knock eject sequence."""
    ci = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in HOME}
    qa = {n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in HOME}
    for n, v in HOME.items():       # start at home pose
        d.qpos[qa[n]] = v
    mujoco.mj_forward(m, d)

    cur = np.zeros(m.nu)
    for n, v in HOME.items():
        cur[ci[n]] = v
    d.ctrl[:] = cur
    spf = max(1, round((1.0 / fps) / m.opt.timestep))
    k = 0
    for _label, targets, secs in EJECT_SEQUENCE:
        nxt = cur.copy()
        for n, v in targets.items():
            nxt[ci[n]] = v
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
    for n, v in HOME.items():
        d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
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
    print(f"printer_cell: nbody={m.nbody} nu={m.nu}  (bed Z-slide + toolhead XY)  -> {out/'printer_cell.png'}")
