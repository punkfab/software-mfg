"""coupling_tool_side.py — tool-side half of the kinematic tool changer.

Carries 3 ball domes at 120° that seat into the arm-side vee-grooves for exact-
constraint, self-centering engagement. Printed-in hemispherical bosses here
(swap for pressed steel balls for a hardened, higher-precision surface). Central
bore aligns with the arm-side EPM / pogo-pin field.

    python parts/coupling_tool_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Pos, Sphere  # noqa: E402

from _coupling import FEATURE_ANGLES, FEATURE_R, PLATE_T, FEATURE_pos, plate_blank  # noqa: E402

BALL_D = 8.0  # coupling ball Ø (dome protrudes a hemisphere above the top face)

part = plate_blank()
for a in FEATURE_ANGLES:
    x, y = FEATURE_pos(FEATURE_R, a)
    part += Pos(x, y, PLATE_T) * Sphere(BALL_D / 2)  # half embedded -> fused dome

if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/coupling_tool_side.step")
    export_stl(part, "exports/coupling_tool_side.stl")
    print("tool-side coupling:", [round(v, 1) for v in part.bounding_box().size])
