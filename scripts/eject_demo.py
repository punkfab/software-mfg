#!/usr/bin/env python3
"""Render the P1S eject-in-place sequence (headless).

cool -> open door -> full-width sweep pushes the part off the front edge, out the
door, into the catch bin -> sweep return -> close door.

    MUJOCO_GL=osmesa python scripts/eject_demo.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from printer_cell import build_model, run_eject  # noqa: E402

W, H, FPS = 640, 480, 30
OUT = ROOT / "exports" / "renders"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    m = build_model()
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.0, -0.05, 0.10]
    cam.distance, cam.azimuth, cam.elevation = 0.9, 120, -22

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an",
         "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(OUT / "eject.mp4")],
        stdin=subprocess.PIPE,
    )
    frames = [0]

    def on_frame():
        renderer.update_scene(d, camera=cam)
        ff.stdin.write(renderer.render().tobytes())
        frames[0] += 1

    run_eject(m, d, on_frame=on_frame, fps=FPS)
    ff.stdin.close()
    ff.wait()

    n = frames[0]
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(OUT / "eject.mp4"),
                    "-vf", "fps=15,scale=480:-1:flags=lanczos", str(OUT / "eject.gif")], check=True)
    for i, t in enumerate([0.30, 0.55, 0.75], 1):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t*n/FPS:.2f}",
                        "-i", str(OUT / "eject.mp4"), "-frames:v", "1",
                        str(OUT / f"eject_still_{i}.png")], check=True)
    print(f"rendered {n} frames -> exports/renders/eject.mp4/.gif + 3 stills")
    return 0


if __name__ == "__main__":
    sys.exit(main())
