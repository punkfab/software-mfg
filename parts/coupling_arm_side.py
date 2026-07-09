"""coupling_arm_side.py — arm-side (master) half of the kinematic tool changer.

Bolts to the SO-101 wrist. The receiver, from centre out:

  * a central passage the tool POST enters (Ø = head + clearance). The actual capture
    is the moving Fidlock shuttle in the canonical part (read-only, composed by
    reference) — our sim abstracts it as an EPM-weld toggle, so here it is just the
    bore the post passes through. [_coupling §2]
  * 3 vee-grooves at FEATURE_ANGLES (UNEQUAL spacing) — the tool balls drop in and
    exact-constrain all 6 DOF in ONE orientation. [_coupling §1]
  * a raised CATCH RING with 3 entry slots — the bayonet receiver: the tool lugs pass
    axially through the slots, then a twist parks them under the ring lip so the prying
    moment rides the rim. [_coupling §3]
  * keyed pogo PIN bores (one set — pressed-in pogo pins meet the tool pads).

    python parts/coupling_arm_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

import _coupling as c  # noqa: E402

C, MN = Align.CENTER, Align.MIN

# A square-section bar rotated 45deg about its radial (X) axis is a diamond; its lower
# vertex cuts a 90deg vee. Place the vertex VEE_DEPTH below the top face.
_zc = (c.PLATE_T - c.VEE_DEPTH) + c.VEE_W / (2 ** 0.5)
_vee = Rot(45, 0, 0) * Box(c.VEE_L, c.VEE_W, c.VEE_W)


def _build():
    # disc with the central post-passage bore (head passes through)
    add = [c.plate_blank(bore=c.LATCH_HEAD_D + c.POCKET_CLR)]

    # raised catch ring at the rim (a lip the tool lugs hook under after the twist)
    if c.RIM_LOCK:
        ring = (Pos(0, 0, c.PLATE_T - 0.2) * Cylinder(c.CATCH_RING_OR, c.CATCH_H + 0.2, align=(C, C, MN))
                - Pos(0, 0, c.PLATE_T - 1.2) * Cylinder(c.CATCH_RING_IR, c.CATCH_H + 2, align=(C, C, MN)))
        add.append(ring)

    part = add[0].fuse(*add[1:]) if len(add) > 1 else add[0]

    # 3 registration vees
    for a in c.FEATURE_ANGLES:
        part -= Rot(0, 0, a) * Pos(c.FEATURE_R, 0, _zc) * _vee

    # 3 bayonet entry slots through the catch ring (radial gaps at the lug angles)
    if c.RIM_LOCK:
        for a in c.LUG_ANGLES:
            part -= Rot(0, 0, a) * Pos((c.CATCH_RING_IR + c.CATCH_RING_OR) / 2, 0, c.PLATE_T + c.CATCH_H / 2) \
                * Box(c.CATCH_RING_OR - c.CATCH_RING_IR + 4, c.LUG_W + 1.5, c.CATCH_H + 2)

    # keyed pogo pin bores (through, for pressed-in pogo pins)
    for b in c.pogo_bores(-1, c.PLATE_T + 2):
        part -= b
    return part


part = _build()   # module-level built solid (repo convention: check_parts reads `part`)


if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/coupling_arm_side.step")
    export_stl(part, "exports/coupling_arm_side.stl")
    import trimesh
    m = trimesh.load("exports/coupling_arm_side.stl")
    print("arm-side coupling:", [round(v, 1) for v in part.bounding_box().size],
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
