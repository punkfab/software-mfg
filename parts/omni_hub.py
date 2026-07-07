"""omni_hub.py — the omni-wheel hub/frame that holds the barrel rollers (TWO staggered rows).
   py cad -> build/omni_hub.stl

Carve-out design: start from a solid blank and SUBTRACT each roller's envelope (barrel + spin
clearance), for all 2*N rollers across both rows, so the rollers spin free by construction.
What remains between pockets holds the off-the-shelf metal pins (tangent, one per roller).
Central bore + horn mount drive it. Reverse-engineered from _omni.py — MEASURE + set params.
"""
import math
import os
import sys

import numpy as np
from build123d import *

sys.path.insert(0, os.path.dirname(__file__))
from _omni import (BARREL_MAX, HALF_L, HORN_BC_D, HORN_N, HORN_SCREW, HUB_BORE,  # noqa: E402
                   HUB_PIN_BORE, MOUNT_R, N_ROLLERS, PIN_SNAP_MOUTH, R_EFF, ROLLER_SAMPLES,
                   ROW_Z, ROWS, rho, roller_center)

CLR = 1.0                                  # roller spin clearance (pocket = barrel + CLR)
CLEAR_R = MOUNT_R + 3.0                     # blank radius: holds pins, stays inside the roller OD
FRAME_H = 2 * (ROW_Z + BARREL_MAX) + 4      # Z height: spans both rows of barrels


def _envelope_roller(clr=CLR):
    """A solid barrel = the roller dilated by clr, used to carve its clearance pocket."""
    zc = HALF_L + clr
    zs = np.linspace(-zc, zc, ROLLER_SAMPLES)
    outer = [(max(0.2, float(rho(z)) + clr), float(z)) for z in zs]
    pts = [(0.2, -zc)] + outer + [(0.2, zc)]
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(*pts, close=True)
            make_face()
        revolve(axis=Axis.Z)
    return p.part


def _build():
    hub = Cylinder(CLEAR_R, FRAME_H)                   # solid blank, axis Z
    cutter = _envelope_roller()
    for row in range(ROWS):
        for i in range(N_ROLLERS):
            (cx, cy, cz), deg = roller_center(i, row)
            hub -= Pos(cx, cy, cz) * Rot(0, 0, deg) * Rot(90, 0, 0) * cutter   # carve pocket

    hub -= Cylinder(HUB_BORE / 2, FRAME_H + 2)         # drive bore
    for k in range(HORN_N):                            # horn bolt circle
        a = 2 * math.pi * k / HORN_N
        hub -= Pos((HORN_BC_D / 2) * math.cos(a), (HORN_BC_D / 2) * math.sin(a), 0) * \
            Cylinder(HORN_SCREW / 2, FRAME_H + 2)
    rad_out = R_EFF + 2 - MOUNT_R                       # throat reaches from the pin out past the OD
    for row in range(ROWS):                            # a SNAP-FIT pin seat per roller
        for i in range(N_ROLLERS):
            (cx, cy, cz), deg = roller_center(i, row)
            frame = Pos(cx, cy, cz) * Rot(0, 0, deg)   # local X = radial out, Y = tangent, Z = axial
            # round seat the pin snaps into (wraps past its centerline -> retains)
            hub -= frame * Rot(90, 0, 0) * Cylinder(HUB_PIN_BORE / 2, 2 * (HALF_L + 8))
            # radial entry throat, narrower than the pin (Z gap = PIN_SNAP_MOUTH): the lips flex,
            # the pin snaps past them into the seat
            hub -= frame * Box(rad_out, 2 * (HALF_L + 8), PIN_SNAP_MOUTH,
                               align=(Align.MIN, Align.CENTER, Align.CENTER))
    return hub


part = _build()   # module-level built solid (repo convention)


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    export_stl(part, "build/omni_hub.stl")
    import trimesh
    m = trimesh.load("build/omni_hub.stl")
    print("omni_hub:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
