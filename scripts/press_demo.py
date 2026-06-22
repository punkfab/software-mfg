#!/usr/bin/env python3
"""Render the C-frame press seating a bearing (headless).

    MUJOCO_GL=osmesa python scripts/press_demo.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from press_cell import build_model, run_press  # noqa: E402

W, H, FPS = 600, 480, 30
OUT = ROOT / "exports" / "renders"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    m = build_model()
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, 0.0, 0.05]
    cam.distance, cam.azimuth, cam.elevation = 0.26, 35, -12

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an",
         "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(OUT / "press.mp4")],
        stdin=subprocess.PIPE,
    )
    frames = [0]

    def on_frame():
        renderer.update_scene(d, camera=cam)
        ff.stdin.write(renderer.render().tobytes())
        frames[0] += 1

    run_press(m, d, on_frame=on_frame, fps=FPS)
    ff.stdin.close()
    ff.wait()
    n = frames[0]
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(OUT / "press.mp4"),
                    "-vf", "fps=15,scale=480:-1:flags=lanczos", str(OUT / "press.gif")], check=True)
    for i, t in enumerate([0.15, 0.55, 0.8], 1):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t*n/FPS:.2f}",
                        "-i", str(OUT / "press.mp4"), "-frames:v", "1",
                        str(OUT / f"press_still_{i}.png")], check=True)
    print(f"rendered {n} frames -> exports/renders/press.mp4/.gif + stills")
    return 0


if __name__ == "__main__":
    sys.exit(main())
