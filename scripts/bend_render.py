#!/usr/bin/env python3
"""Render the actual bent wire produced by the wirebender cell.

    MUJOCO_GL=osmesa python scripts/bend_render.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from wirebender_cell import simulate  # noqa: E402

OUT = ROOT / "exports" / "renders"
WIRE_R = 0.0008  # 1.6 mm wire -> 0.8 mm radius, in metres


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    r = simulate("staple")
    pts = np.asarray(r["points"]) * 1e-3            # mm -> m
    pts -= pts.mean(0)                              # centre it

    spec = mujoco.MjSpec()
    spec.worldbody.add_light(pos=[0.05, -0.05, 0.2], dir=[-0.2, 0.2, -1])
    for i in range(len(pts) - 1):
        g = spec.worldbody.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_CAPSULE
        g.fromto = [*pts[i], *pts[i + 1]]
        g.size = [WIRE_R, 0, 0]
        g.rgba = [0.7, 0.72, 0.75, 1]
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    renderer = mujoco.Renderer(m, height=420, width=560)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0]
    cam.distance, cam.azimuth, cam.elevation = 0.07, 90, -75   # near top-down
    renderer.update_scene(d, camera=cam)
    png = OUT / "bent_wire.png"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                    "-s", "560x420", "-i", "-", "-frames:v", "1", str(png)],
                   input=renderer.render().tobytes(), check=True)
    print(f"bent wire: {r['length']} mm, {r['n_bends']} bends -> {png.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
