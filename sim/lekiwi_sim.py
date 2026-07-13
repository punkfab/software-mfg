"""lekiwi_sim.py — MuJoCo bench for the EXACT LeKiwi (3-omni base + SO-101 arm).

Composes the thirdparty LeKiwi checkout BY REFERENCE (read-only): loads its URDF, repairs
the meshes into build/lekiwi/ (the 3 omni wheels are 314k faces -> over MuJoCo's limit; we
decimate a copy, never touching the checkout), adds a floor + a FREE base joint, and drives
it holonomically.

Fidelity notes (honest):
  - Geometry, masses, inertias, joint frames = the LeKiwi URDF, unmodified.
  - The base is a FREE body, so the arm's reaction really does tip/wobble it (the point).
  - The omni wheels are single-body meshes (no rollers), so they can't slip-model. The
    holonomic drive is therefore an IDEAL planar controller on the base; the 3 wheels spin
    at the LeKiwi inverse-kinematic speeds for display + logging. Swap in per-roller contact
    or anisotropic friction later for traction fidelity.

  python sim/lekiwi_sim.py            # interactive viewer (needs a display)
  python sim/lekiwi_sim.py --demo     # headless: drive + arm reach -> build/lekiwi/lekiwi.gif
  python sim/lekiwi_sim.py --selftest # build + step + assert stands, no window (CI)
"""
import glob
import math
import os
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

_MODE = ("reach" if "--reach" in sys.argv else "demo" if "--demo" in sys.argv
         else "selftest" if "--selftest" in sys.argv else "interactive")
os.environ.setdefault("MUJOCO_GL", "glfw" if _MODE == "interactive" else "osmesa")

import numpy as np  # noqa: E402
import mujoco  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))                                 # so bridge/ (shared kinematics) imports
LEKIWI = Path("/home/dan/sandbox/thirdparty/LeKiwi/URDF")     # read-only thirdparty checkout
BUILD = ROOT / "build" / "lekiwi"
WHEEL_JOINTS = ["ST3215_Servo_Motor-v1-2_Revolute-60",        # the 3 drive wheels (from the URDF)
                "ST3215_Servo_Motor-v1-1_Revolute-62",
                "ST3215_Servo_Motor-v1_Revolute-64"]


@dataclass
class Params:
    dt: float = 2e-3
    wheel_r: float = 0.055        # omni wheel radius (m)
    base_r: float = 0.12          # wheel pitch radius (m)
    wheel_mu: float = 0.06        # omni ideal: wheels support weight, ~no traction resistance
    drive_kp: float = 700.0       # base planar position gain (N/m, N·m/rad)
    drive_kd: float = 170.0
    settle_s: float = 1.2


# --- compose the thirdparty model by reference: repair meshes into our build dir ------------
def prepare_meshes(force=False):
    (BUILD / "meshes").mkdir(parents=True, exist_ok=True)
    urdf = BUILD / "LeKiwi.urdf"
    if urdf.exists() and not force:
        return urdf
    import trimesh
    import fast_simplification
    for f in glob.glob(str(LEKIWI / "meshes" / "*.stl")):
        m = trimesh.load(f)
        if len(m.faces) > 60000:                                 # over MuJoCo's decoder limit
            v, fc = fast_simplification.simplify(m.vertices, m.faces,
                                                 target_reduction=1 - 25000 / len(m.faces))
            m = trimesh.Trimesh(v, fc)
        m.export(str(BUILD / "meshes" / Path(f).name))
    shutil.copy(str(LEKIWI / "LeKiwi.urdf"), str(urdf))
    return urdf


def build_model(p: Params, obstacle=None):
    urdf = prepare_meshes()
    spec = mujoco.MjSpec.from_file(str(urdf))
    spec.option.timestep = p.dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    wb = spec.worldbody
    fl = wb.add_geom(); fl.name = "floor"; fl.type = mujoco.mjtGeom.mjGEOM_PLANE
    fl.size = [0, 0, 0.05]; fl.rgba = [0.3, 0.32, 0.36, 1]; fl.friction = [p.wheel_mu, 0.02, 0.001]
    wb.add_light(pos=[0.3, -0.3, 1.0], dir=[-0.3, 0.3, -1])
    if obstacle:                                           # a static box (e.g. a printer) to drive up to
        g = wb.add_geom(); g.name = "obstacle"; g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = list(obstacle["half"]); g.pos = list(obstacle["pos"]); g.rgba = [0.7, 0.72, 0.76, 1]
    base = spec.body("base_plate_layer1-v5")
    base.add_freejoint()
    base.pos = [0, 0, 0.13]
    # position actuators on the 6 arm joints (so the viewer gets a slider per joint)
    for j in spec.joints:
        if j.name.startswith("STS3215_03a"):
            a = spec.add_actuator(); a.name = "arm_" + j.name.split("_")[-1]
            a.target = j.name; a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.gainprm[0] = 12.0; a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.biasprm[1] = -12.0; a.biasprm[2] = -0.6
    model = spec.compile()
    return model


# --- LeKiwi holonomic kinematics: body (vx, vy, wz) -> 3 wheel angular speeds (rad/s) --------
def body_to_wheels(vx, vy, wz, p: Params, angles_deg=(300.0, 180.0, 60.0)):
    """Holonomic inverse kinematics, DELEGATED to bridge/omni_kinematics so the sim and the real
    hardware driver share one algorithm (sim-is-a-cache-of-reality). Falls back to the identical
    inline formula if the shared module isn't importable, so the sim stays self-contained."""
    try:
        from bridge.omni_kinematics import OmniGeometry, body_to_wheels as _b2w
        geo = OmniGeometry(wheel_r_mm=p.wheel_r * 1000, base_r_mm=p.base_r * 1000,
                           azimuths_deg=tuple(angles_deg))
        return list(_b2w(geo, vx, vy, wz))
    except Exception:
        out = []
        for b in angles_deg:
            b = math.radians(b)
            v_wheel = -math.sin(b) * vx + math.cos(b) * vy + p.base_r * wz   # wheel rim linear speed
            out.append(v_wheel / p.wheel_r)
        return out


def _ids(model):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_plate_layer1-v5")
    base_dof = model.jnt_dofadr[model.body_jntadr[bid]]        # free-joint dof start
    wheel_dofs = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]
                  for j in WHEEL_JOINTS]
    return bid, base_dof, wheel_dofs


def tilt_deg(model, data, bid):
    return float(np.degrees(np.arccos(np.clip(data.xmat[bid].reshape(3, 3)[2, 2], -1, 1))))


def run(p: Params, scenario, steps, render=False, cam=None, obstacle=None):
    model = build_model(p, obstacle=obstacle)
    data = mujoco.MjData(model)
    bid, base_dof, wheel_dofs = _ids(model)
    for _ in range(int(p.settle_s / p.dt)):                    # settle onto the wheels
        mujoco.mj_step(model, data)
    renderer = mujoco.Renderer(model, 480, 640) if render else None
    frames, log = [], {k: [] for k in ("t", "x", "y", "yaw", "tilt", "wheel0")}
    for k in range(steps):
        t = k * p.dt
        (tx, ty, tyaw), arm = scenario(t)
        # ideal holonomic base drive: PD to the target planar pose (world x, y, yaw)
        x, y = data.xpos[bid][0], data.xpos[bid][1]
        yaw = math.atan2(data.xmat[bid].reshape(3, 3)[1, 0], data.xmat[bid].reshape(3, 3)[0, 0])
        vx, vy, wz = data.qvel[base_dof], data.qvel[base_dof + 1], data.qvel[base_dof + 5]
        data.qfrc_applied[base_dof + 0] = p.drive_kp * (tx - x) - p.drive_kd * vx
        data.qfrc_applied[base_dof + 1] = p.drive_kp * (ty - y) - p.drive_kd * vy
        dyaw = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
        data.qfrc_applied[base_dof + 5] = 0.5 * p.drive_kp * dyaw - 0.5 * p.drive_kd * wz
        w = body_to_wheels(vx, vy, wz, p)          # LeKiwi wheel speeds (logged for reference)
        # arm setpoints
        for i in range(min(model.nu, len(arm))):
            data.ctrl[i] = arm[i]
        mujoco.mj_step(model, data)
        for key, val in (("t", t), ("x", x), ("y", y), ("yaw", yaw),
                         ("tilt", tilt_deg(model, data, bid)), ("wheel0", w[0])):
            log[key].append(val)
        if renderer and k % int(1 / 60 / p.dt) == 0:
            renderer.update_scene(data, cam); frames.append(renderer.render())
    return model, data, log, frames


# --- scenarios ------------------------------------------------------------------------------
def drive_and_reach(t):
    """Holonomic tour: forward, strafe, spin — while the arm lifts from stow into a reach."""
    if t < 2.0:    tgt = (0.4 * (t / 2.0), 0.0, 0.0)                 # forward
    elif t < 4.0:  tgt = (0.4, 0.3 * (t - 2.0) / 2.0, 0.0)          # strafe
    elif t < 6.0:  tgt = (0.4, 0.3, math.radians(90) * (t - 4.0) / 2.0)  # spin in place
    else:          tgt = (0.4, 0.3, math.radians(90))
    reach = min(1.0, max(0.0, (t - 1.0) / 2.5))                     # 0->1 arm extension
    arm = [0.0, -0.7 * reach, 0.9 * reach, -0.4 * reach, 0.0, 0.0]  # shoulder/elbow/wrist reach
    return tgt, arm


# --- modes ----------------------------------------------------------------------------------
def selftest():
    if not LEKIWI.exists():
        print(f"SKIP: LeKiwi checkout not found at {LEKIWI} (compose-by-reference; clone it there)")
        return 0
    p = Params()
    model, data, log, _ = run(p, drive_and_reach, steps=int(7.5 / p.dt))
    xf, yf, yawf = log["x"][-1], log["y"][-1], log["yaw"][-1]
    # holonomic: translated in x AND y (not just one axis) AND spun, all while upright + finite
    ok = (np.all(np.isfinite(data.qpos)) and max(log["tilt"]) < 25.0
          and xf > 0.30 and yf > 0.18 and abs(yawf) > 1.2)
    print(f"lekiwi: {model.nbody} bodies, {model.njnt} joints (free base + 3 wheels + 6 arm), "
          f"{model.nu} arm actuators")
    print(f"  holonomic drive: x={xf:.2f}m, y={yf:.2f}m, yaw={np.degrees(yawf):.0f}deg "
          f"(translated BOTH axes + spun); peak tilt {max(log['tilt']):.1f}deg (arm reaction); "
          f"finite={bool(np.all(np.isfinite(data.qpos)))}")
    print("PASS: exact LeKiwi (3-omni + SO-101) drives holonomically and stays upright"
          if ok else "FAIL: unstable / not holonomic")
    return 0 if ok else 1


def demo():
    p = Params()
    cam = mujoco.MjvCamera(); cam.azimuth, cam.elevation, cam.distance = 125, -20, 1.1
    cam.lookat = [0.25, 0.15, 0.18]
    model, data, log, frames = run(p, drive_and_reach, steps=int(6.5 / p.dt), render=True, cam=cam)
    import imageio
    BUILD.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(str(BUILD / "lekiwi.gif"), frames, fps=30)
    print(f"drove ({log['x'][-1]:.2f},{log['y'][-1]:.2f})m, peak tilt {max(log['tilt']):.1f}deg "
          f"-> {BUILD/'lekiwi.gif'} ({len(frames)} frames)")


def reach():
    """Drive-up-and-reach: reach_plan finds the closest SAFE standoff for a printer pick, the base
    drives there (clearing the printer box), then the arm reaches out over the bed. -> build/lekiwi/reach.gif"""
    import reach_plan as rp
    target, printer, base_m, arm_m = rp.printer_pick()
    pl = rp.plan(target, printer, base_m, arm_m)
    if not pl.reachable:
        print("plan infeasible:", pl.reason); return 1
    bx, by, byaw = pl.base_pose
    obstacle = {"pos": [printer.cx, printer.cy, printer.top_z / 2],
                "half": [printer.hx, printer.hy, printer.top_z / 2]}
    drive_T = 4.0

    def scenario(t):
        if t < drive_T:                                    # ramp the base pose origin -> standoff
            f = t / drive_T
            return (bx * f, by * f, byaw * f), [0.0] * 6
        r = min(1.0, (t - drive_T) / 2.5)                  # then extend the arm out over the bed
        return (bx, by, byaw), [0.0, -0.7 * r, 0.9 * r, -0.4 * r, 0.0, 0.0]

    p = Params()
    cam = mujoco.MjvCamera(); cam.azimuth, cam.elevation, cam.distance = 140, -22, 1.7
    cam.lookat = [0.5 * (bx + printer.cx), 0.5 * (by + printer.cy), 0.2]
    model, data, log, frames = run(p, scenario, steps=int(7.5 / p.dt), render=True, cam=cam,
                                   obstacle=obstacle)
    import imageio
    BUILD.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(str(BUILD / "reach.gif"), frames, fps=30)
    xf, yf = log["x"][-1], log["y"][-1]
    print(f"planned standoff ({bx:.2f},{by:.2f}) yaw {math.degrees(byaw):.0f}° -> reached "
          f"({xf:.2f},{yf:.2f}), err {math.hypot(xf-bx, yf-by)*1000:.0f} mm; printer clearance "
          f"{pl.footprint_clearance_m*1000:.0f} mm; peak tilt {max(log['tilt']):.1f}° -> "
          f"{BUILD/'reach.gif'} ({len(frames)} frames)")
    return 0


def interactive():
    from mujoco import viewer         # bind `viewer` (not `mujoco`) so the global stays visible
    p = Params()
    model = build_model(p)
    data = mujoco.MjData(model)
    viewer.launch(model, data)


if __name__ == "__main__":
    sys.exit({"selftest": selftest, "demo": demo, "interactive": interactive,
              "reach": reach}[_MODE]() or 0)
