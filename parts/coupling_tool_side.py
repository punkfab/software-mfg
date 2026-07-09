"""coupling_tool_side.py — tool-side half of the kinematic tool changer.

The half that bolts to a process tool (shear, glue, gripper). Carries, from centre out:

  * a central mushroom POST — the Fidlock catch target. A steel pin down its bore is
    the magnet target; the head's flat underside is what the arm-side barb hooks (axial
    load rides the mechanical catch, not the magnet — fails locked). [_coupling §2]
  * 3 pockets for BOUGHT Ø8 steel balls at FEATURE_ANGLES (UNEQUAL spacing) — the Maxwell
    registration, seats in exactly one orientation so the pogo set + jaw point the right
    way. Metal balls (not printed domes) = hardened + repeatable, and no printed sphere
    to mangle the STL — cf. parts/omni_hub.py's bought-pin approach. [_coupling §1]
  * 3 rim LUGS at RIM_R — hand the prying moment to the circumference for the rim lock.
    [_coupling §3; sim/coupling_statics.py says whether they earn their place]
  * a keyed pogo pad cluster (one set — the single orientation means no duplicates).

    python parts/coupling_tool_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

import _coupling as c  # noqa: E402

C, MN = Align.CENTER, Align.MIN


def _build():
    part = c.plate_blank(bore=0)   # no central bore — the post occupies the centre

    # Union incrementally with `+` (each piece overlaps the plate, so it stays one solid).
    # OCC's multi-arg .fuse(*spheres) turns tangent embedded balls into hollow shells;
    # adding one at a time is the proven pattern (cf. the prior tool-side revision).

    # central Fidlock post: neck then head (barb); sink each into its mate by >=1mm.
    part += Pos(0, 0, c.PLATE_T - 1.0) * Cylinder(c.LATCH_NECK_D / 2, c.LATCH_NECK_H + 1.0,
                                                  align=(C, C, MN))
    part += Pos(0, 0, c.PLATE_T + c.LATCH_NECK_H - 1.0) \
        * Cylinder(c.LATCH_HEAD_D / 2, c.LATCH_HEAD_H + 1.0, align=(C, C, MN))

    # 3 rim lugs (raised tabs near the OD; the arm catch ring hooks over them on a twist).
    if c.RIM_LOCK:
        for a in c.LUG_ANGLES:
            part += Rot(0, 0, a) * Pos(c.RIM_R, 0, c.PLATE_T - 1.5) \
                * Box(c.LUG_SPAN, c.LUG_W, c.LUG_T + 1.5, align=(C, C, MN))

    # cuts (all subtractive; overshoot faces):
    #  - 3 pockets for the bought steel balls (glued in; protrude ~3mm to seat in the vees)
    for a in c.FEATURE_ANGLES:
        x, y = c.pos_at(c.FEATURE_R, a)
        part -= Pos(x, y, c.PLATE_T - c.BALL_POCKET_DEPTH) \
            * Cylinder(c.BALL_POCKET_D / 2, c.BALL_POCKET_DEPTH + 0.5, align=(C, C, MN))
    #  - steel-pin bore down the post (the magnet target)
    part -= Pos(0, 0, -1) * Cylinder(c.LATCH_PIN_D / 2,
                                     c.PLATE_T + c.LATCH_NECK_H + c.LATCH_HEAD_H + 2,
                                     align=(C, C, MN))
    #  - shallow pogo pad recesses
    for b in c.pogo_bores(c.PLATE_T - 1.5, 3.0):
        part -= b
    return part


part = _build()   # module-level built solid (repo convention: check_parts reads `part`)


if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/coupling_tool_side.step")
    export_stl(part, "exports/coupling_tool_side.stl")
    import trimesh
    m = trimesh.load("exports/coupling_tool_side.stl")
    print("tool-side coupling:", [round(v, 1) for v in part.bounding_box().size],
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
