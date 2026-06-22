#!/usr/bin/env python3
"""Bender -> arm geometric handoff: the REAL bent wire, presented and sheared.

The wire bender cell produces a staple (its own forward model); we place that
actual polyline at the datum and run the arm's tool-change + present + shear on
it — closing the loop between the two cells in one scene.

    MUJOCO_GL=osmesa python scripts/cell_handoff_demo.py
"""

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "scripts"))
import ik  # noqa: E402
from toolchanger import DATUM_POS, build_model  # noqa: E402
from toolchange_demo import SEGMENTS  # noqa: E402
from wirebender_cell import staple_in_workcell  # noqa: E402

W, H, FPS = 640, 480, 30
OUT = ROOT / "exports" / "renders"
WIRE_Z = 0.086  # on the work table; the present pose brings the blade down to it


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    # the bender's real output, placed under where the shear blade closes
    wire = staple_in_workcell([DATUM_POS[0] + 0.03, DATUM_POS[1]], WIRE_Z)
    m = build_model(wire_points=wire)
    d = mujoco.MjData(m)

    tip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "moving_jaw_so101_v1")
    tool_b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "shear_tool")
    dock = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "dock")
    shear_ci = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "shear")
    mujoco.mj_forward(m, d)

    def weld_in_place():
        ip, iq, rp, rq = np.zeros(3), np.zeros(4), np.zeros(3), np.zeros(4)
        mujoco.mju_negPose(ip, iq, d.xpos[tip], d.xquat[tip])
        mujoco.mju_mulPose(rp, rq, ip, iq, d.xpos[tool_b], d.xquat[tool_b])
        m.eq_data[dock, 3:6], m.eq_data[dock, 6:10], m.eq_data[dock, 10] = rp, rq, 1

    renderer = mujoco.Renderer(m, height=H, width=W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.235, 0.0, 0.092]
    cam.distance, cam.azimuth, cam.elevation = 0.3, 100, -16

    ff = subprocess.Popen(
        ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-", "-an",
         "-vcodec", "libx264", "-pix_fmt", "yuv420p", str(OUT / "cell_handoff.mp4")],
        stdin=subprocess.PIPE,
    )
    spf = max(1, round((1.0 / FPS) / m.opt.timestep))
    cur, prev_weld, k, n = np.zeros(m.nu), False, 0, 0

    for _label, target, shear, weld, secs in SEGMENTS:
        nxt = cur.copy()
        if target is not None:
            scratch = mujoco.MjData(m)
            scratch.qpos[:] = d.qpos
            nxt[: ik.ARM_DOFS], _ = ik.solve_ik(m, scratch, tip, np.asarray(target))
        nxt[shear_ci] = shear
        if weld and not prev_weld:
            weld_in_place()
        d.eq_active[dock] = 1 if weld else 0
        prev_weld = weld
        for i in range(max(1, int(secs / m.opt.timestep))):
            d.ctrl[:] = cur + (nxt - cur) * ((i + 1) / max(1, int(secs / m.opt.timestep)))
            mujoco.mj_step(m, d)
            if k % spf == 0:
                renderer.update_scene(d, camera=cam)
                ff.stdin.write(renderer.render().tobytes())
                n += 1
            k += 1
        cur = nxt

    ff.stdin.close()
    ff.wait()
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(OUT / "cell_handoff.mp4"),
                    "-vf", "fps=15,scale=480:-1:flags=lanczos", str(OUT / "cell_handoff.gif")], check=True)
    for i, t in enumerate([0.5, 0.62, 0.72], 1):
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{t*n/FPS:.2f}",
                        "-i", str(OUT / "cell_handoff.mp4"), "-frames:v", "1",
                        str(OUT / f"cell_handoff_still_{i}.png")], check=True)
    print(f"rendered {n} frames; real bent wire ({len(wire)} pts) presented + sheared "
          f"-> exports/renders/cell_handoff.mp4")
    return 0


if __name__ == "__main__":
    sys.exit(main())
