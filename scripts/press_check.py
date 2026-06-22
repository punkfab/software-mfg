#!/usr/bin/env python3
"""Gate: the C-frame press seats the bearing AND is self-reacting.

The press force must be high and the arm/mount must feel almost none of it
(the force loops inside the C-frame).
"""

import sys
from pathlib import Path

import mujoco

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from press_cell import PRESS_FORCE, SEAT_DEPTH, build_model, run_press  # noqa: E402


def main() -> int:
    m = build_model()
    d = mujoco.MjData(m)
    rec = {}
    run_press(m, d, record=rec)
    ratio = rec["mount_force"] / max(rec["press_force"], 1)

    problems = []
    if not (SEAT_DEPTH * 1000 - 1 < rec["insertion"] * 1000 < SEAT_DEPTH * 1000 + 2):
        problems.append(f"bearing not seated ({rec['insertion']*1000:.1f} mm, want ~{SEAT_DEPTH*1000:.0f})")
    if rec["press_force"] < 0.8 * PRESS_FORCE:
        problems.append(f"press force too low ({rec['press_force']:.0f} N)")
    if ratio > 0.1:
        problems.append(f"not self-reacting (arm feels {ratio*100:.0f}% of the press force)")

    print(f"seated {rec['insertion']*1000:.1f} mm | press {rec['press_force']:.0f} N | "
          f"arm holds {rec['mount_force']:.1f} N | self-reaction ratio {ratio:.3f}")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: C-frame seats the bearing with high force; the arm feels ~none of it")
    return 0


if __name__ == "__main__":
    sys.exit(main())
