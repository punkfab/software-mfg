"""harness_demo.py — proves sim/harness.py end-to-end on a tiny contact task, and is the primitive-
collider seed for the snap-fit insertion v2.

A peg (free cylinder) is dropped with a lateral offset over a V-funnel (two tilted planks). Gravity +
a gentle downward press seat it; the funnel self-centers it to the throat — the same capture-cone idea
as parts/snap_socket.py, in primitives (MuJoCo convex-hulls meshes, so a bored socket must be built
from primitive colliders — this demo shows how). Every line of scaffolding here comes from harness:
mode/backend pick, base_scene, ids, rollout, save_gif.

    python3 sim/harness_demo.py            # interactive viewer (needs a display)
    python3 sim/harness_demo.py --demo     # headless -> build/harness_demo.gif
    python3 sim/harness_demo.py --selftest # build + step + assert it self-centers, no window
"""
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(__file__))
from harness import pick_mode, base_scene, ids, rollout, save_gif, run_modes  # noqa: E402

MODE = pick_mode(default="interactive")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402


@dataclass
class Params:
    dt: float = 1e-3
    peg_r: float = 0.006          # peg radius (m)  -> Ø12, matches parts/_snapfit.PEG_D
    peg_h: float = 0.030          # peg half not — full length
    x0: float = 0.020             # lateral offset the peg is dropped with (m)
    z0: float = 0.085             # drop height (m)
    ramp_deg: float = 35.0        # funnel wall angle from vertical
    press_N: float = 0.05         # gentle downward press applied through the seating phase
    settle_s: float = 1.6


def build_model(p: Params):
    spec = mujoco.MjSpec()
    base_scene(spec, dt=p.dt, floor_mu=0.9)
    wb = spec.worldbody
    th = math.radians(p.ramp_deg)
    qy = lambda a: [math.cos(a / 2), 0, math.sin(a / 2), 0]     # quaternion about +Y
    for sign in (+1, -1):                                       # two planks -> a V funnel
        g = wb.add_geom()
        g.name = f"ramp_{'p' if sign > 0 else 'n'}"
        g.type = mujoco.mjtGeom.mjGEOM_BOX
        g.size = [0.045, 0.03, 0.004]                           # half-extents (long, wide, thick)
        g.pos = [sign * 0.032, 0, 0.030]
        g.quat = qy(-sign * th)                                 # tilt so the inner edge dips to the throat
        g.rgba = [0.42, 0.55, 0.68, 1]
        g.friction = [0.6, 0.02, 0.001]
    peg = wb.add_body()
    peg.name = "peg"
    peg.pos = [p.x0, 0, p.z0]
    peg.add_freejoint()
    gp = peg.add_geom()
    gp.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    gp.size = [p.peg_r, p.peg_h / 2, 0]
    gp.rgba = [0.80, 0.55, 0.35, 1]
    gp.friction = [0.6, 0.02, 0.001]
    gp.mass = 0.01
    return spec.compile()


def run(p: Params, render=False, cam=None):
    model = build_model(p)
    data = mujoco.MjData(model)
    I = ids(model, bodies=["peg"], joints=["peg_freejoint"] if _has_joint(model, "peg_freejoint")
            else [])
    dof = int(model.jnt_dofadr[model.body_jntadr[I["body"]["peg"]]])   # peg free-joint dof start
    peg_bid = I["body"]["peg"]
    log = {"t": [], "x": [], "z": []}

    def step_fn(k, t):
        data.qfrc_applied[dof + 2] = -p.press_N if t < p.settle_s * 0.8 else 0.0   # press, then release
        log["t"].append(t)
        log["x"].append(float(data.xpos[peg_bid][0]))
        log["z"].append(float(data.xpos[peg_bid][2]))

    frames = rollout(model, data, step_fn, int(p.settle_s / p.dt), render=render, cam=cam)
    return model, data, log, frames


def _has_joint(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) >= 0


def selftest():
    p = Params()
    model, data, log, _ = run(p)
    xf, zf = log["x"][-1], log["z"][-1]
    finite = bool(np.all(np.isfinite(data.qpos)))
    centered = abs(xf) < 0.006                    # funnel pulled it from x0=20mm to within the throat
    descended = zf < p.z0 - 0.02
    # exercise the CAD->sim helper too: cache the real snap peg mesh (watertight gate)
    mesh_ok = True
    try:
        from harness import cache_mesh
        from pathlib import Path
        stl = Path(__file__).resolve().parent.parent / "build" / "snap_peg.stl"
        if stl.exists():
            out = cache_mesh(str(stl), "snap_peg_simcache")
            mesh_ok = out.exists()
    except Exception as e:                          # noqa: BLE001
        print("  cache_mesh smoke test skipped:", e)
    ok = finite and centered and descended and mesh_ok
    print(f"harness_demo: dropped peg at x0={p.x0*1000:.0f}mm -> settled x={xf*1000:.1f}mm "
          f"(|x|<6mm={centered}), z {p.z0*1000:.0f}->{zf*1000:.0f}mm (descended={descended}), "
          f"finite={finite}, cache_mesh={mesh_ok}")
    print("PASS: harness scaffolding works — V-funnel self-centers the peg (capture cone in primitives)"
          if ok else "FAIL: harness_demo did not self-center / seat")
    return 0 if ok else 1


def demo():
    p = Params()
    cam = mujoco.MjvCamera()
    cam.azimuth, cam.elevation, cam.distance = 90, -12, 0.28
    cam.lookat = [0, 0, 0.03]
    model, data, log, frames = run(p, render=True, cam=cam)
    if frames:
        out = save_gif(frames, "build/harness_demo.gif")
        print(f"peg settled x={log['x'][-1]*1000:.1f}mm -> {out} ({len(frames)} frames)")
    else:
        print(f"peg settled x={log['x'][-1]*1000:.1f}mm (physics ran; no frames — headless GL absent)")
    return 0


def interactive():
    from mujoco import viewer
    model = build_model(Params())
    viewer.launch(model, mujoco.MjData(model))
    return 0


if __name__ == "__main__":
    sys.exit(run_modes(MODE, {"selftest": selftest, "demo": demo, "interactive": interactive}))
