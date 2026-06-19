#!/usr/bin/env python3
"""Render the SO-101 running a scripted motion to a video + stills (headless).

Offscreen OSMesa render of a short waypoint trajectory under position control,
piped to ffmpeg. Produces exports/renders/so101_demo.{mp4,gif} and a few PNG
stills. Run with: MUJOCO_GL=osmesa python scripts/so101_render.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SCENE = ROOT / "sim" / "so101" / "scene.xml"
OUT = ROOT / "exports" / "renders"
W, H, FPS = 640, 480, 30

# Waypoint targets (rad): [pan, lift, elbow, wrist_flex, wrist_roll, gripper].
# (seconds_to_reach, target) — ramped linearly, position control tracks.
WAYPOINTS = [
    (0.6, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),     # settle at home
    (1.2, [1.0, -0.7, 0.9, 0.6, 0.0, 1.5]),    # reach right, open gripper
    (0.8, [1.0, -0.7, 0.9, 0.6, 2.5, 0.0]),    # roll wrist, close gripper
    (1.6, [-1.2, -0.4, 0.6, -0.5, -2.5, 1.0]), # sweep left
    (1.2, [0.0, 0.0, 0.0, 0.0, 0.0, 0.2]),     # return home
]


def trajectory(model):
    """Yield per-step ctrl targets by linearly ramping between waypoints."""
    dt = model.opt.timestep
    cur = np.array(WAYPOINTS[0][1], dtype=float)
    for secs, tgt in WAYPOINTS:
        tgt = np.array(tgt, dtype=float)
        n = max(1, int(secs / dt))
        for i in range(n):
            yield cur + (tgt - cur) * ((i + 1) / n)
        cur = tgt


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=H, width=W)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.08, 0.0, 0.12]
    cam.distance, cam.azimuth, cam.elevation = 0.75, 150, -20

    steps_per_frame = max(1, round((1.0 / FPS) / m.opt.timestep))
    mp4 = OUT / "so101_demo.mp4"

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an",
         "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(mp4)],
        stdin=subprocess.PIPE,
    )

    nframes = 0
    for k, ctrl in enumerate(trajectory(m)):
        d.ctrl[:] = ctrl
        mujoco.mj_step(m, d)
        if k % steps_per_frame == 0:
            renderer.update_scene(d, camera=cam)
            ff.stdin.write(renderer.render().tobytes())
            nframes += 1
    ff.stdin.close()
    ff.wait()

    # Derive a gif + 3 evenly-spaced stills from the mp4.
    gif = OUT / "so101_demo.gif"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(mp4),
                    "-vf", "fps=15,scale=480:-1:flags=lanczos", str(gif)], check=True)
    for i, t in enumerate([0.18, 0.5, 0.82], 1):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t*nframes/FPS:.2f}",
                        "-i", str(mp4), "-frames:v", "1", str(OUT / f"so101_still_{i}.png")],
                       check=True)

    print(f"rendered {nframes} frames @ {FPS}fps")
    print(f"  {mp4.relative_to(ROOT)}")
    print(f"  {gif.relative_to(ROOT)}")
    print(f"  stills: so101_still_1..3.png in {OUT.relative_to(ROOT)}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
