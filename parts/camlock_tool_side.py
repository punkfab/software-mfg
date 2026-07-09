"""camlock_tool_side.py — tool half of the servo-driven central draw-lock changer.

Bolts to a process tool. From the centre out:
  * a central POST (Ø14 shaft) carrying 3 interrupted ACME-thread LUGS at 120deg — the
    cam ring's catches hook under these and draw the post home. [_camlock §2]
  * 3 pockets for BOUGHT Ø8 steel balls at EQUAL 120deg — the kinematic locator (metal
    balls: hardened, repeatable, no printed-sphere STL artifact). [_camlock §1]
  * a keyed pogo pad cluster (power/data across the joint).

    python parts/camlock_tool_side.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Cylinder, Pos  # noqa: E402

import _camlock as c  # noqa: E402

C, MN = Align.CENTER, Align.MIN


def _build():
    part = c.plate_blank(bore=0)     # disc + 3 mount bolts; post occupies the centre

    # central draw post (sink 1mm into the plate so it welds)
    part += Pos(0, 0, c.PLATE_T - 1.0) * Cylinder(c.POST_SHAFT_D / 2, c.POST_H + 1.0,
                                                  align=(C, C, MN))
    # 3 interrupted ACME-thread lugs (flat sectors; the cam ramp does the drawing). r_in < the
    # shaft radius so each lug welds into the post.
    for a in c.LUG_ANGLES:
        part += c.annular_sector(c.POST_SHAFT_D / 2 - 1.0, c.R_LUG, c.PLATE_T + c.LUG_Z,
                                 c.LUG_T, a, c.LUG_ARC)

    # 3 kinematic ball pockets (bought steel balls glued in, protrude ~3mm)
    for a in c.FEATURE_ANGLES:
        x, y = c.pos_at(c.FEATURE_R, a)
        part -= Pos(x, y, c.PLATE_T - c.BALL_POCKET_DEPTH) \
            * Cylinder(c.BALL_POCKET_D / 2, c.BALL_POCKET_DEPTH + 0.5, align=(C, C, MN))

    # keyed pogo pad recesses
    for b in c.pogo_bores(c.PLATE_T - 1.5, 3.0):
        part -= b
    return part


part = _build()


if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/camlock_tool_side.step")
    export_stl(part, "exports/camlock_tool_side.stl")
    import trimesh
    m = trimesh.load("exports/camlock_tool_side.stl")
    print("camlock tool-side:", [round(v, 1) for v in part.bounding_box().size],
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
