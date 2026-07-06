"""glue_body.py — printed housing for the hot-glue applicator (the "build" part).

Carries the coupling mount (top), the glue-stick feed channel (back->front), the
feed-motor mount + drive-wheel cavity (-Y face), the idler pivot, and the front-face
mount for the thermal-break nozzle bracket. The bought melt cartridge + nozzle sit
beyond the front, held off by the bracket so the print never touches the hot zone.

    python parts/glue_body.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

from _glue import (BODY_H, BODY_X, BODY_Y, BOLT_CLR, BOLT_R, BORE_D, COUPLING_OD,  # noqa: E402
                   COUPLING_T, DRIVE_X, FEED_BOLT_SP, FEED_HOLE, FEED_PILOT_D,
                   FEED_SHAFT_CLR, FRONT_MOUNT_SP, FRONT_TAP, IDLER_PIN_D, IDLER_PIN_X,
                   IDLER_Z, MOUNT_ANGLES, STICK_D, STICK_Z, WHEEL_CAV_D, pos)

C, MN = Align.CENTER, Align.MIN

# main body block (base z=0) + coupling plate on top (sunk 1 mm so the union is solid)
part = Box(BODY_X, BODY_Y, BODY_H, align=(C, C, MN))
part += Pos(0, 0, BODY_H - 1) * Cylinder(COUPLING_OD / 2, COUPLING_T + 1, align=(C, C, MN))

# central bore + 3 coupling mount holes (down through the top plate)
part -= Pos(0, 0, BODY_H - 2) * Cylinder(BORE_D / 2, COUPLING_T + 6, align=(C, C, MN))
for a in MOUNT_ANGLES:
    x, y = pos(BOLT_R, a)
    part -= Pos(x, y, BODY_H - 2) * Cylinder(BOLT_CLR / 2, COUPLING_T + 6, align=(C, C, MN))

# glue-stick channel: a through bore along X at mid-height (back entry -> front exit)
part -= Pos(0, 0, STICK_Z) * Rot(0, 90, 0) * Cylinder(STICK_D / 2, BODY_X + 4, align=(C, C, C))

# feed drive on the -Y face: pilot recess + shaft/hub pass-through + drive-wheel cavity + 2 M3
face_y = -BODY_Y / 2
part -= Pos(DRIVE_X, face_y - 0.01, STICK_Z) * Rot(-90, 0, 0) * Cylinder(FEED_PILOT_D / 2, 4, align=(C, C, MN))
part -= Pos(DRIVE_X, face_y - 1, STICK_Z) * Rot(-90, 0, 0) * Cylinder(FEED_SHAFT_CLR / 2, BODY_Y + 2, align=(C, C, MN))
part -= Pos(DRIVE_X, 0, STICK_Z) * Rot(-90, 0, 0) * Cylinder(WHEEL_CAV_D / 2, WHEEL_CAV_D, align=(C, C, C))
for dz in (-FEED_BOLT_SP / 2, FEED_BOLT_SP / 2):
    part -= Pos(DRIVE_X, face_y - 1, STICK_Z + dz) * Rot(-90, 0, 0) * Cylinder(FEED_HOLE / 2, 9, align=(C, C, MN))

# idler pivot: a through bore along Y, above the channel
part -= Pos(IDLER_PIN_X, 0, IDLER_Z) * Rot(-90, 0, 0) * Cylinder(IDLER_PIN_D / 2, BODY_Y + 4, align=(C, C, MN))

# front face: 2 self-tap bosses (either side of the channel) for the nozzle bracket
front_x = BODY_X / 2
for dy in (-FRONT_MOUNT_SP / 2, FRONT_MOUNT_SP / 2):
    part -= Pos(front_x - 4, dy, STICK_Z) * Rot(0, 90, 0) * Cylinder(FRONT_TAP / 2, 10, align=(C, C, C))

if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/glue_body.step")
    export_stl(part, "exports/glue_body.stl")
    print("glue_body:", [round(v, 1) for v in part.bounding_box().size])
