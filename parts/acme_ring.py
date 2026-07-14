"""acme_ring.py — the hex ACME LOCKING RING for the servoless tool changer.

Captive on the arm piece (spins free), the dock's wrench slot grabs the hex OD to ground it while the
wrist twists the tool onto/off the internal ACME thread. Internal 4-start ACME (from _acme_changer,
validated in sim/toolchanger_acme.py); a top inward flange is the captive retainer.

    py/bin/python parts/acme_ring.py -> build/acme_ring.stl
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build123d import (Align, Cylinder, Pos, RegularPolygon, export_stl, extrude)
import _acme_changer as A

RING_L = 14.0            # ring height
FLANGE_T = 3.0          # top retaining flange (captured under the arm-piece shoulder)
FLANGE_BORE = 30.0      # flange inner Ø — the arm-piece hub passes; the lip catches the shoulder


def part():
    hexblank = extrude(RegularPolygon(A.RING_AF / 2, 6, major_radius=False), RING_L)
    # internal 4-start ACME over the LOWER (RING_L - FLANGE_T); the top flange keeps its small bore
    ring = hexblank - A.acme_female_cutter(length=RING_L - FLANGE_T + 1)
    # the retaining flange: a plain FLANGE_BORE hole through the top FLANGE_T (smaller than the thread)
    ring -= Pos(0, 0, RING_L - FLANGE_T) * Cylinder(FLANGE_BORE / 2, FLANGE_T + 1,
                                                    align=(Align.CENTER, Align.CENTER, Align.MIN))
    return ring


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = part()
    export_stl(p, "build/acme_ring.stl")
    import trimesh
    m = trimesh.load("build/acme_ring.stl")
    print("acme_ring:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
