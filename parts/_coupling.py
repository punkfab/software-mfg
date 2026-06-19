"""_coupling.py — shared parameters for the kinematic tool-changer coupling.

A Maxwell coupling (3 balls in 3 vee-grooves) gives exact-constraint, self-
centering seating to ~microns (CONCEPT.md §5.2). The arm-side carries the vees,
the tool-side carries the balls. Both share this blank: a disc with a central
bore (EPM / pogo-pin pass-through) and a 3-bolt mounting pattern.

`_`-prefixed so scripts/check_parts.py skips it; the two halves import it.
"""

import math

from build123d import Align, Cylinder, Pos

C, MN = Align.CENTER, Align.MIN

PLATE_OD = 50.0      # coupling disc outer Ø
PLATE_T = 6.0        # disc thickness
BORE_D = 12.0        # central bore (electropermanent magnet + pogo pins)
FEATURE_R = 16.0     # radius of the 3 coupling features (balls / vees)
BOLT_R = 21.0        # mounting bolt circle
BOLT_CLR = 3.4       # M3 clearance
MOUNT_ANGLES = (30, 150, 270)    # 3 mounting bolts
FEATURE_ANGLES = (90, 210, 330)  # 3 coupling features, 120° apart, offset from bolts


def plate_blank():
    """Disc with central bore + 3 M3 mounting holes; base at z=0, top at z=PLATE_T."""
    p = Cylinder(PLATE_OD / 2, PLATE_T, align=(C, C, MN))
    p -= Pos(0, 0, -1) * Cylinder(BORE_D / 2, PLATE_T + 2, align=(C, C, MN))
    for a in MOUNT_ANGLES:
        x, y = FEATURE_pos(BOLT_R, a)
        p -= Pos(x, y, -1) * Cylinder(BOLT_CLR / 2, PLATE_T + 2, align=(C, C, MN))
    return p


def FEATURE_pos(r, angle_deg):
    return r * math.cos(math.radians(angle_deg)), r * math.sin(math.radians(angle_deg))
