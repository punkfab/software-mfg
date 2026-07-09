"""camlock_arm_side.py — arm (master) half of the servo-driven draw-lock changer.

Bolts to the SO-101 wrist. From the centre out:
  * a central through BORE the cam ring rotates in; the cam's top flange lands on this
    plate's top face and reacts the draw load. [_camlock §2]
  * 3 vee-grooves at EQUAL 120deg — the kinematic locator the tool balls seat into. The
    120deg vees also take the cam's LOCKING TORQUE reaction (geometric anti-spin), so the
    tool doesn't spin while the servo cinches it. [_camlock §1; sim/camlock_statics.py]
  * 2 bracket holes by the drive-arm side for the servo mount + a keyed pogo pin set.

    python parts/camlock_arm_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

import _camlock as c  # noqa: E402

C, MN = Align.CENTER, Align.MIN

# 45deg-diamond vee cutter -> a 90deg groove, apex VEE_DEPTH below the top face
_zc = (c.PLATE_T - c.VEE_DEPTH) + c.VEE_W / (2 ** 0.5)
_vee = Rot(45, 0, 0) * Box(c.VEE_L, c.VEE_W, c.VEE_W)


def _build():
    part = c.plate_blank(bore=c.ARM_BORE_D)      # disc + bolts + central cam bore

    for a in c.FEATURE_ANGLES:                    # 3 registration vees at 120deg
        part -= Rot(0, 0, a) * Pos(c.FEATURE_R, 0, _zc) * _vee

    for b in c.pogo_bores(-1, c.PLATE_T + 2):     # keyed pogo pins (through)
        part -= b

    # 2 servo-bracket mount holes by the drive-arm side (angle 0), clear of the vees
    for a in (-10.0, 10.0):
        x, y = c.pos_at(30.0, a)
        part -= Pos(x, y, -1) * Cylinder(c.BRACKET_SCREW_CLR / 2, c.PLATE_T + 2, align=(C, C, MN))
    return part


part = _build()


if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/camlock_arm_side.step")
    export_stl(part, "exports/camlock_arm_side.stl")
    import trimesh
    m = trimesh.load("exports/camlock_arm_side.stl")
    print("camlock arm-side:", [round(v, 1) for v in part.bounding_box().size],
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
