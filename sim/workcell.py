"""workcell.py — compose the SO-101 into a manufacturing workcell.

Builds on the vendored SO-101 snapshot *from the outside* using MuJoCo's MjSpec
API: it loads sim/so101/scene.xml (so meshdir resolves correctly) and adds a
work table and a fiducial datum — the local geometric reference where precision
lives (CONCEPT.md principle #1). The snapshot is never edited.

Phase 2 extends this same builder with a tool rack + self-actuated shear + a
toggleable weld for the tool changer.

Usage:
    from workcell import build_model, DATUM_POS
    model = build_model()
Run directly to render a preview still:
    MUJOCO_GL=osmesa python sim/workcell.py
"""

from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "so101" / "scene.xml"

# The work datum: where the bender presents wire / where parts are placed. Fine
# positioning targets this local reference, not the arm's global pose.
DATUM_POS = np.array([0.22, 0.0, 0.10])
TABLE_TOP_Z = 0.085  # table surface just below the datum


def build_spec() -> "mujoco.MjSpec":
    spec = mujoco.MjSpec.from_file(str(SCENE))
    wb = spec.worldbody

    # Work table (a slab in front of the arm).
    table = wb.add_geom()
    table.name = "worktable"
    table.type = mujoco.mjtGeom.mjGEOM_BOX
    table.size = [0.11, 0.16, TABLE_TOP_Z / 2]
    table.pos = [DATUM_POS[0], 0.0, TABLE_TOP_Z / 2]
    table.rgba = [0.33, 0.34, 0.40, 1.0]

    # Fixture nub at the datum — gives the arm a physical thing to register against.
    fix = wb.add_geom()
    fix.name = "datum_fixture"
    fix.type = mujoco.mjtGeom.mjGEOM_BOX
    fix.size = [0.012, 0.012, (DATUM_POS[2] - TABLE_TOP_Z) / 2]
    fix.pos = [DATUM_POS[0], DATUM_POS[1], (DATUM_POS[2] + TABLE_TOP_Z) / 2]
    fix.rgba = [0.7, 0.2, 0.2, 1.0]

    # Fiducial marker site at the datum origin (visual reference, no collision).
    fid = wb.add_site()
    fid.name = "work_datum"
    fid.type = mujoco.mjtGeom.mjGEOM_SPHERE
    fid.size = [0.006, 0, 0]
    fid.pos = DATUM_POS.tolist()
    fid.rgba = [1.0, 0.85, 0.0, 1.0]

    return spec


def build_model() -> "mujoco.MjModel":
    return build_spec().compile()


if __name__ == "__main__":
    import subprocess

    m = build_model()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    out = ROOT / "exports" / "renders"
    out.mkdir(parents=True, exist_ok=True)

    renderer = mujoco.Renderer(m, height=480, width=640)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.14, 0.0, 0.10]
    cam.distance, cam.azimuth, cam.elevation = 0.8, 150, -20
    renderer.update_scene(d, camera=cam)
    png = out / "workcell.png"
    # write via ffmpeg (single rawvideo frame -> png)
    subprocess.run(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", "640x480", "-i", "-", "-frames:v", "1", str(png)],
        input=renderer.render().tobytes(), check=True,
    )
    print(f"workcell: nbody={m.nbody} ngeom={m.ngeom}  datum={DATUM_POS.tolist()}")
    print(f"preview -> {png.relative_to(ROOT)}")
