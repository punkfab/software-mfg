"""coupling_arm_side.py — arm-side half of the kinematic tool changer.

The receiver: the coupling disc with 3 radial vee-grooves at 120°. The tool-side
balls drop into these vees and self-center (exact constraint). The central bore
takes the electropermanent magnet that pulls the tool in and holds it with zero
standing power.

    python parts/coupling_arm_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Box, Pos, Rot  # noqa: E402

from _coupling import FEATURE_ANGLES, FEATURE_R, PLATE_T, FEATURE_pos, plate_blank  # noqa: E402

VEE_DEPTH = 3.0   # groove depth into the top face
VEE_W = 8.0       # cutter cross-section (sets the 90° vee opening width)
VEE_L = 20.0      # radial groove length

# A square-section bar rotated 45° about its radial (X) axis becomes a diamond;
# its lower vertex cuts a 90° vee. Place the vertex VEE_DEPTH below the top face.
_zc = (PLATE_T - VEE_DEPTH) + VEE_W / (2 ** 0.5)
_vee = Rot(45, 0, 0) * Box(VEE_L, VEE_W, VEE_W)

part = plate_blank()
for a in FEATURE_ANGLES:
    part -= Rot(0, 0, a) * Pos(FEATURE_R, 0, _zc) * _vee

if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/coupling_arm_side.step")
    export_stl(part, "exports/coupling_arm_side.stl")
    print("arm-side coupling:", [round(v, 1) for v in part.bounding_box().size])
