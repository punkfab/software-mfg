#!/usr/bin/env python3
"""Render the full tool-change + shear sequence (headless).

approach rack -> couple (weld/EPM on) -> lift tool -> present at datum ->
actuate shear (cut the wire proxy) -> reopen -> return to rack -> decouple.

Drives the arm with the IK helper, ramps position-control targets per segment,
and toggles the weld via data.eq_active. Writes exports/renders/toolchange.{mp4,gif}.

    MUJOCO_GL=osmesa python scripts/toolchange_demo.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import ik  # noqa: E402
from toolchanger import DATUM_POS, MATE_TARGET, TOOL_HANG, build_model  # noqa: E402

W, H, FPS = 640, 480, 30
OUT = ROOT / "exports" / "renders"

# Segments: (label, jaw_target | None, shear, weld_active, seconds)
APPROACH = MATE_TARGET + np.array([0, 0, 0.06])
CARRY = MATE_TARGET + np.array([0, 0, 0.07])
PRESENT = DATUM_POS + np.array([0, 0, TOOL_HANG + 0.012])  # tool blade down at the wire
SEGMENTS = [
    ("approach rack",   APPROACH,     0.0, False, 1.0),
    ("descend to tool", MATE_TARGET,  0.0, False, 0.9),
    ("couple (EPM on)", MATE_TARGET,  0.0, True,  0.5),
    ("lift tool",       CARRY,        0.0, True,  1.0),
    ("present at datum", PRESENT,     0.0, True,  1.3),
    ("shear (cut)",     PRESENT,      0.9, True,  0.6),
    ("reopen",          PRESENT,      0.0, True,  0.5),
    ("return to rack",  CARRY,        0.0, True,  1.1),
    ("seat on rack",    MATE_TARGET,  0.0, True,  0.7),
    ("decouple",        MATE_TARGET,  0.0, False, 0.4),
    ("retract",         APPROACH,     0.0, False, 0.9),
]


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    m = build_model()
    d = mujoco.MjData(m)
    scratch = mujoco.MjData(m)

    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    tool_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shear_tool")
    dock = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "dock")
    shear_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "shear")
    mujoco.mj_forward(m, d)

    def weld_in_place():
        """Capture the current end<-tool relative pose into the weld (zero snap)."""
        ipos, iquat = np.zeros(3), np.zeros(4)
        rpos, rquat = np.zeros(3), np.zeros(4)
        mujoco.mju_negPose(ipos, iquat, d.xpos[tip], d.xquat[tip])
        mujoco.mju_mulPose(rpos, rquat, ipos, iquat, d.xpos[tool_body], d.xquat[tool_body])
        m.eq_data[dock, 0:3] = 0
        m.eq_data[dock, 3:6] = rpos
        m.eq_data[dock, 6:10] = rquat
        m.eq_data[dock, 10] = 1

    renderer = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.12, 0.09, 0.10]
    cam.distance, cam.azimuth, cam.elevation = 0.52, 110, -22

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an",
         "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(OUT / "toolchange.mp4")],
        stdin=subprocess.PIPE,
    )

    steps_per_frame = max(1, round((1.0 / FPS) / m.opt.timestep))
    cur = np.zeros(m.nu)
    nframes = 0
    prev_weld = False

    for label, target, shear, weld, secs in SEGMENTS:
        nxt = cur.copy()
        if target is not None:
            scratch.qpos[:] = d.qpos
            q_arm, _ = ik.solve_ik(m, scratch, tip, np.asarray(target))
            nxt[:ik.ARM_DOFS] = q_arm
        nxt[shear_ci] = shear
        if weld and not prev_weld:        # rising edge: weld where we are, no snap
            weld_in_place()
        d.eq_active[dock] = 1 if weld else 0
        prev_weld = weld

        nsteps = max(1, int(secs / m.opt.timestep))
        start = cur.copy()
        for i in range(nsteps):
            d.ctrl[:] = start + (nxt - start) * ((i + 1) / nsteps)
            mujoco.mj_step(m, d)
            if (i % steps_per_frame) == 0:
                renderer.update_scene(d, camera=cam)
                ff.stdin.write(renderer.render().tobytes())
                nframes += 1
        cur = nxt

    ff.stdin.close()
    ff.wait()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(OUT / "toolchange.mp4"),
                    "-vf", "fps=15,scale=480:-1:flags=lanczos", str(OUT / "toolchange.gif")],
                   check=True)
    # a few stills across the sequence
    for i, t in enumerate([0.22, 0.45, 0.6, 0.85], 1):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t*nframes/FPS:.2f}",
                        "-i", str(OUT / "toolchange.mp4"), "-frames:v", "1",
                        str(OUT / f"toolchange_still_{i}.png")], check=True)

    print(f"rendered {nframes} frames -> exports/renders/toolchange.mp4/.gif + 4 stills")
    return 0


if __name__ == "__main__":
    sys.exit(main())
