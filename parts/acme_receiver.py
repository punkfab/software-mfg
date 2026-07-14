"""acme_receiver.py — the TOOL-SIDE receiver for the servoless ACME tool changer.

Every effector presents this: a central external 4-start ACME post (the ring threads onto it and
draws the tool up) + 3 spherical ball-locator seats OUTBOARD of the ring (they seat into the arm's
TPU vee plate — the kinematic seat that fixes orientation) + the tool-mount contract on the back
(Ø12 bore + 3×M3 @ R21, same as coupling_tool_side, so the shear/glue/gripper bodies bolt on).

    py/bin/python parts/acme_receiver.py -> build/acme_receiver.stl
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build123d import (Align, Cylinder, Pos, export_stl)
import _acme_changer as A

PLATE_D = 74.0          # receiver disc Ø (carries the outboard ball seats)
PLATE_T = 6.0          # disc thickness (hosts the tool-mount bolts)
THREAD_L = 10.0        # engaged external thread length
BALL_SINK = 0.45       # ball sits this fraction into its seat -> protrudes to meet the vee


def part():
    # back plate with the tool-mount contract (Ø12 bore + 3×M3)
    plate = Cylinder(PLATE_D / 2, PLATE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    plate -= Pos(0, 0, -1) * Cylinder(A.TOOL_BORE_D / 2, PLATE_T + 2,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    for a in A.TOOL_BOLT_ANG:
        x, y = A.TOOL_BOLT_R * math.cos(math.radians(a)), A.TOOL_BOLT_R * math.sin(math.radians(a))
        plate -= Pos(x, y, -1) * Cylinder(A.M3_CLR / 2, PLATE_T + 2,
                                          align=(Align.CENTER, Align.CENTER, Align.MIN))
    # central external ACME post rising from the plate face (root sinks into the plate -> fuses)
    post = Pos(0, 0, PLATE_T - 1) * A.acme_male(length=THREAD_L + 1)
    part = plate + post
    # 3 ball-locator SEATS OUTBOARD (r = BALL_PCD/2 > ring corners): a Ø8 cylindrical pocket the BOUGHT
    # steel ball glues into (protrudes to meet the arm's TPU vee). A cylinder, not a printed sphere,
    # keeps the STL clean (no sphere-pole artifacts) — same "bought ball" approach as the coupling.
    seat_depth = 0.55 * A.BALL_D
    for a in (90, 210, 330):
        x, y = A.BALL_PCD / 2 * math.cos(math.radians(a)), A.BALL_PCD / 2 * math.sin(math.radians(a))
        part -= Pos(x, y, PLATE_T - seat_depth) * Cylinder(A.BALL_D / 2, seat_depth + 1,
                                                           align=(Align.CENTER, Align.CENTER, Align.MIN))
    return part


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = part()
    export_stl(p, "build/acme_receiver.stl")
    import trimesh
    m = trimesh.load("build/acme_receiver.stl")
    print("acme_receiver:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
