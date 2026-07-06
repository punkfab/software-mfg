#!/usr/bin/env python3
"""Render the assembled hot-glue tool (printed body + thermal-break nozzle bracket).

    MUJOCO_GL=osmesa python scripts/glue_demo.py

Loads the exported STLs and places the bracket at the body's front face so the
assembly reads as one tool. Run `make parts` first if the STLs are missing.
"""

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "parts"))
from _glue import BODY_X, STICK_Z  # noqa: E402

EXPORTS = ROOT / "exports"
OUT = ROOT / "exports" / "renders"
W, H = 640, 480
_G = mujoco.mjtGeom


def add_part(spec, name, stl, pos, rgba):
    m = spec.add_mesh()
    m.name = name
    m.file = str(EXPORTS / stl)
    m.scale = [0.001, 0.001, 0.001]     # mm -> m
    b = spec.worldbody.add_body()
    b.pos = pos
    g = b.add_geom()
    g.type = _G.mjGEOM_MESH
    g.meshname = name
    g.rgba = rgba


def main() -> int:
    for stl in ("glue_body.stl", "glue_nozzle_bracket.stl"):
        if not (EXPORTS / stl).exists():
            print(f"missing {stl} — run `make parts` first")
            return 1
    spec = mujoco.MjSpec()
    spec.option.gravity = [0, 0, 0]
    spec.worldbody.add_light(pos=[0.1, -0.15, 0.25], dir=[-0.3, 0.4, -1])
    add_part(spec, "body", "glue_body.stl", [0, 0, 0], [0.30, 0.55, 0.75, 1])
    add_part(spec, "bracket", "glue_nozzle_bracket.stl",
             [BODY_X / 2000.0, 0, STICK_Z / 1000.0], [0.5, 0.52, 0.56, 1])
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.012, 0, 0.028]
    cam.distance, cam.azimuth, cam.elevation = 0.14, 35, -18
    renderer.update_scene(d, camera=cam)
    OUT.mkdir(parents=True, exist_ok=True)
    from PIL import Image
    png = OUT / "glue_tool.png"
    Image.fromarray(renderer.render()).save(png)
    print(f"rendered assembled glue tool -> {png.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
