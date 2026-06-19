#!/usr/bin/env python3
"""Live interactive viewer for the P1S printer cell, looping the eject sequence.

The printer cell is built in code (MjSpec), so there's no .xml for
`mujoco.viewer --mjcf` to open — this launches the passive viewer on the built
model and replays cool -> open door -> toolhead-knock -> bed drop on a loop.

    DISPLAY=:0 python scripts/printer_view.py        # run via `!` for a window
"""

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from printer_cell import build_model, run_eject  # noqa: E402

FPS = 60


def main() -> int:
    m = build_model()
    d = mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m, d) as viewer:
        # frame a nice initial view of the cell
        viewer.cam.lookat[:] = [0, -0.05, 0.12]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 0.9, 120, -22

        def on_frame():
            if not viewer.is_running():
                raise KeyboardInterrupt
            viewer.sync()
            time.sleep(1.0 / FPS)

        try:
            while viewer.is_running():
                mujoco.mj_resetData(m, d)        # part back on the bed
                run_eject(m, d, on_frame=on_frame, fps=FPS)
                for _ in range(FPS):             # pause between cycles
                    on_frame()
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
