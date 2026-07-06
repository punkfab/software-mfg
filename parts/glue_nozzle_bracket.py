"""glue_nozzle_bracket.py — thermal-break standoff that holds the bought melt cartridge.

Bolts to the body's front face and clamps the bought hot-end barrel a THERMAL_GAP
in front of the print, so conduction into the printed body is starved (thin necks +
air gap). Aligns the cartridge bore with the stick-channel exit; the nozzle protrudes
past the front. Printed, one solid.

Frame: mounts in the Y-Z plane at x=0 (against the body front), stands off in +X.

    python parts/glue_nozzle_bracket.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

from _glue import (BOLT_CLR, FRONT_MOUNT_SP, HOTEND_BARREL_D, THERMAL_GAP)  # noqa: E402

C, MN = Align.CENTER, Align.MIN

STANDOFF_L = THERMAL_GAP + 10.0   # x depth: gap + clamp ring
BRK_Y = 30.0                      # width
BRK_Z = 24.0                      # height
BREAK_SLOT = 6.0                  # thermal-break slot width (leaves thin necks)

# standoff block (x from 0 at the body face, out to the clamp)
part = Box(STANDOFF_L, BRK_Y, BRK_Z, align=(MN, C, C))

# barrel clamp bore along X (holds the bought cartridge, aligned to the channel exit)
part -= Pos(-1, 0, 0) * Rot(0, 90, 0) * Cylinder(HOTEND_BARREL_D / 2, STANDOFF_L + 2, align=(C, C, MN))

# 2 mounting clearance holes (along X) to the body's front self-tap bosses
for dy in (-FRONT_MOUNT_SP / 2, FRONT_MOUNT_SP / 2):
    part -= Pos(-1, dy, 0) * Rot(0, 90, 0) * Cylinder(BOLT_CLR / 2, STANDOFF_L + 2, align=(C, C, MN))

# thermal break: cut two slots from top and bottom in the mid-span, leaving thin necks
for dz in (BRK_Z / 2, -BRK_Z / 2):
    part -= Pos(THERMAL_GAP / 2 + 2, 0, dz) * Box(BREAK_SLOT, BRK_Y - 8, BRK_Z / 2, align=(C, C, C))

if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/glue_nozzle_bracket.step")
    export_stl(part, "exports/glue_nozzle_bracket.stl")
    print("glue_nozzle_bracket:", [round(v, 1) for v in part.bounding_box().size])
