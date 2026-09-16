"""harness.py — shared scaffolding for MuJoCo sims, so a new mechanism sim is params + physics, not
boilerplate. Factored from the repeated pattern across sim/*.py: mode+backend selection, the CAD->mesh
decimation/watertight cache, floor+light scene, position actuators, the render-rollout->gif loop, and
name->id/dof lookups.

USAGE (the ordering matters — MUJOCO_GL must be set BEFORE `import mujoco`):

    from harness import pick_mode
    MODE = pick_mode(default="interactive", extra=("reach",))   # sets MUJOCO_GL from the mode
    import mujoco                                                # now safe to import
    from harness import base_scene, cache_mesh, rollout, save_gif, save_fig, ids

This module imports mujoco/trimesh/etc. LAZILY (inside functions) so it can be imported before the GL
backend is chosen.
"""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"

_MODES = ("interactive", "demo", "selftest", "check")


def pick_mode(argv=None, default="interactive", extra=()):
    """Return the run mode from argv and set MUJOCO_GL appropriately. Call BEFORE importing mujoco.

    The rule (learned the hard way): only force a GL backend when we will actually render.
      - interactive -> glfw (a window)
      - selftest / check -> leave MUJOCO_GL UNSET. These are physics-only, and forcing 'osmesa' on a
        box without libOSMesa makes `import mujoco` crash at import — so the CI gate must never depend
        on a GL library it doesn't use.
      - a rendering mode (demo, or a sim-specific `extra`) -> setdefault 'osmesa' (the repo's headless
        backend; a user/CI MUJOCO_GL always wins). If GL is genuinely absent, rollout() degrades to
        no frames with an actionable message rather than a cryptic PyOpenGL crash.
    `extra` adds sim-specific modes; the first matching --flag wins, else `default`."""
    argv = sys.argv if argv is None else argv
    mode = default
    for m in (*_MODES, *extra):
        if f"--{m}" in argv:
            mode = m
            break
    if mode == "interactive":
        os.environ.setdefault("MUJOCO_GL", "glfw")
    elif mode not in ("selftest", "check"):          # a rendering mode
        os.environ.setdefault("MUJOCO_GL", "osmesa")
    # selftest/check: leave MUJOCO_GL unset so `import mujoco` can't fail on a missing GL lib
    return mode


# --- CAD -> sim mesh pipeline ---------------------------------------------------------------
def cache_mesh(source, name, *, max_faces=60000, decimate_to=25000, out_dir=BUILD, force=False):
    """Ensure a sim-ready STL exists for `name` and return its Path. `source` is either a path to an
    STL or a build123d solid (exported here). Verifies watertight (MuJoCo computes mesh inertia from
    volume, so a leaky mesh gives garbage inertia) and decimates anything over `max_faces` to
    ~`decimate_to` (MuJoCo's mesh decoder chokes above ~60k faces). Centralizes the LeKiwi wheel trick
    and the build123d watertight gate so no sim re-implements them."""
    import trimesh
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    dst = out / f"{name}.stl"
    if dst.exists() and not force and not hasattr(source, "export"):
        pass  # cached STL on disk is fine
    if hasattr(source, "wrapped") or hasattr(source, "export") or type(source).__module__.startswith("build123d"):
        from build123d import export_stl                       # a build123d solid
        export_stl(source, str(dst))
        m = trimesh.load(str(dst))
    else:
        m = trimesh.load(str(source))
    if not m.is_watertight:
        print(f"WARN cache_mesh[{name}]: mesh not watertight — MuJoCo inertia will be unreliable")
    if len(m.faces) > max_faces:
        import fast_simplification
        v, f = fast_simplification.simplify(m.vertices, m.faces,
                                            target_reduction=1 - decimate_to / len(m.faces))
        m = trimesh.Trimesh(v, f)
        print(f"cache_mesh[{name}]: decimated to {len(m.faces)} faces")
    m.export(str(dst))
    return dst


def prep_dir(src_glob, out_meshdir, *, max_faces=60000, decimate_to=25000):
    """Decimate/copy a whole directory of meshes (a URDF checkout) into out_meshdir. Returns the dir."""
    import glob
    out = Path(out_meshdir)
    out.mkdir(parents=True, exist_ok=True)
    for f in glob.glob(str(src_glob)):
        cache_mesh(f, Path(f).stem, max_faces=max_faces, decimate_to=decimate_to, out_dir=out)
    return out


# --- MJCF scene helpers (MjSpec) ------------------------------------------------------------
def base_scene(spec, *, dt=2e-3, floor=True, floor_mu=0.8, light=True):
    """Apply the common option/floor/light block to a MjSpec: implicitfast integrator, a grey ground
    plane with the given tangential friction, and one key light. Returns the spec for chaining."""
    import mujoco
    spec.option.timestep = dt
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    wb = spec.worldbody
    if floor:
        g = wb.add_geom()
        g.name = "floor"
        g.type = mujoco.mjtGeom.mjGEOM_PLANE
        g.size = [0, 0, 0.05]
        g.rgba = [0.3, 0.32, 0.36, 1]
        g.friction = [floor_mu, 0.02, 0.001]
    if light:
        wb.add_light(pos=[0.3, -0.3, 1.0], dir=[-0.3, 0.3, -1])
    return spec


def position_actuators(spec, joint_filter, *, kp=12.0, kv=0.6, prefix="act_"):
    """Add a position actuator to every joint whose name passes `joint_filter(name)->bool`, so the
    viewer's Control pane gets one slider per joint. Returns the list of actuator names added."""
    import mujoco
    added = []
    for j in spec.joints:
        if joint_filter(j.name):
            a = spec.add_actuator()
            a.name = prefix + j.name
            a.target = j.name
            a.trntype = mujoco.mjtTrn.mjTRN_JOINT
            a.gainprm[0] = kp
            a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
            a.biasprm[1] = -kp
            a.biasprm[2] = -kv
            added.append(a.name)
    return added


# --- run / render / plot --------------------------------------------------------------------
def ids(model, *, bodies=(), joints=(), geoms=()):
    """name->id (+ joint dof/qpos addresses) lookups in one call. Returns a dict with keys
    'body','joint','geom' (name->id) and 'dof','qpos' (joint name->address). Kills the per-sim
    mj_name2id boilerplate."""
    import mujoco
    o = mujoco.mjtObj
    out = {"body": {}, "joint": {}, "geom": {}, "dof": {}, "qpos": {}}
    for n in bodies:
        out["body"][n] = mujoco.mj_name2id(model, o.mjOBJ_BODY, n)
    for n in joints:
        jid = mujoco.mj_name2id(model, o.mjOBJ_JOINT, n)
        out["joint"][n] = jid
        out["dof"][n] = int(model.jnt_dofadr[jid])
        out["qpos"][n] = int(model.jnt_qposadr[jid])
    for n in geoms:
        out["geom"][n] = mujoco.mj_name2id(model, o.mjOBJ_GEOM, n)
    return out


def rollout(model, data, step_fn, steps, *, render=False, cam=None, w=640, h=480, fps=30):
    """Step the sim `steps` times. step_fn(k, t) runs BEFORE each mj_step (apply ctrl/forces + log via
    a closure). If render, capture ~fps frames into a list. Returns the frames list (empty if not
    rendering). The one render-rollout loop every MuJoCo sim was hand-writing."""
    import mujoco
    dt = model.opt.timestep
    renderer = None
    if render:
        try:
            renderer = mujoco.Renderer(model, h, w)
        except Exception as e:                        # noqa: BLE001 - headless GL may be absent
            print(f"rollout: rendering unavailable ({type(e).__name__}) — no frames captured. "
                  f"Install a headless GL backend (e.g. `sudo apt install libosmesa6`) or run on an "
                  f"EGL/GPU host; physics ran normally.")
    frames, every = [], max(1, int(1.0 / fps / dt))
    for k in range(steps):
        step_fn(k, k * dt)
        mujoco.mj_step(model, data)
        if renderer and k % every == 0:
            renderer.update_scene(data, cam)
            frames.append(renderer.render())
    if renderer:
        renderer.close()
    return frames


def save_gif(frames, path, fps=30):
    import imageio
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(str(p), frames, fps=fps)
    return p


def save_fig(fig, path, dpi=110):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(p), dpi=dpi)
    return p


def run_modes(mode, table):
    """Dispatch: call table[mode]() and exit with its int return (or 0). Standardizes the __main__
    tail so a sim ends with `run_modes(MODE, {"selftest": selftest, "demo": demo, ...})`."""
    fn = table.get(mode)
    if fn is None:
        print(f"harness: no handler for mode '{mode}' (have: {', '.join(table)})")
        return 2
    return fn() or 0
