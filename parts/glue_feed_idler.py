"""glue_feed_idler.py — spring-loaded pinch idler for the glue-stick feed.

A small lever: pivots on a pin through the body (parts/glue_body.py idler bore),
and a concave cradle presses the stick down onto the knurled drive wheel. A torsion
spring (or a rubber band on the tail) supplies the pinch force. Printed, one solid.

    python parts/glue_feed_idler.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

from _glue import IDLER_PIN_D, STICK_D  # noqa: E402

C, MN = Align.CENTER, Align.MIN

LEVER_L = 26.0    # length (pivot end -> stick cradle -> spring tail)
LEVER_W = 8.0     # width (along the pin)
LEVER_H = 9.0     # height
PIN_CLR = IDLER_PIN_D + 0.4   # rotates freely on the pin
PIVOT_X = 4.0     # pivot from the pivot end
CRADLE_X = 15.0   # stick cradle position

# lever bar (base z=0, starts x=0)
part = Box(LEVER_L, LEVER_W, LEVER_H, align=(MN, C, MN))

# pivot bore along Y near the pivot end
part -= Pos(PIVOT_X, 0, LEVER_H / 2) * Rot(-90, 0, 0) * Cylinder(PIN_CLR / 2, LEVER_W + 2, align=(C, C, C))

# stick cradle: a concave groove across the top (a cylinder cut along X)
part -= Pos(CRADLE_X, 0, LEVER_H) * Rot(0, 90, 0) * Cylinder(STICK_D / 2, 14, align=(C, C, C))

# spring seat: a shallow blind pocket on top of the tail (holds a torsion spring leg)
part -= Pos(LEVER_L - 4, 0, LEVER_H - 2) * Cylinder(2.0, 4, align=(C, C, MN))

if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/glue_feed_idler.step")
    export_stl(part, "exports/glue_feed_idler.stl")
    print("glue_feed_idler:", [round(v, 1) for v in part.bounding_box().size])
