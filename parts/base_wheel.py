"""base_wheel.py — 3D-printed drive wheel for a 2x STS3215 diff-drive base.
   py cad -> build/base_wheel.stl

A solid wheel that bolts to an STS3215 servo horn (the servo drives it directly in wheel
mode). Two O-ring grooves on the rim seat rubber O-rings as tyres — cheap, high-grip, and
printable in one material (no TPU tyre needed). Radius 35 mm matches sim/mobile_base.py's
diff-drive design (low deck -> low CG). The horn interface is fully parametric: **measure
your STS3215 horn and set HORN_* before printing** — defaults are plausible, not gospel.

Grip comes from the O-rings, not the plastic: size ORING_CS to your O-ring stock so it sits
~1 mm proud of the rim. Mount: seat the servo's round horn in the hub recess, screw the wheel
to it through the HORN bolt circle, retain with the central shaft screw (reach it through the
outer counterbore).
"""
import os

from build123d import *

# --- wheel body ---
WHEEL_D = 70.0        # outer diameter (mm); r=35 matches the diff-drive model
WHEEL_W = 24.0        # width along the axle
RIM_FILLET = 1.5      # soften the outer edges (printability)

# --- O-ring tyres (grip) ---
ORING_CS = 3.0        # O-ring cross-section stock; groove seats it ~1mm proud
N_GROOVES = 2
GROOVE_INSET = 6.0    # first groove this far from each face

# --- STS3215 horn interface (from the ST-3215-C047 datasheet, §11 accessories horn:
#     horn OD Ø19.95, hub Ø9, bolt circle Ø14, 4× Ø3.2 (M3), 25T/Ø5.9 spline, M3×6 screw) ---
HORN_RECESS_D = 20.5  # pocket the Ø19.95 horn disc registers into (+ ~0.5 fit)
HORN_RECESS_DEPTH = 3.0  # datasheet horn is ~2.1 mm thick (round disc) + hub
CENTER_BORE = 9.2     # clears the horn's Ø9 hub boss
CENTER_CBORE_D = 7.0  # outer-face counterbore so you can reach the central M3×6 retaining screw
CENTER_CBORE_DEPTH = 6.0
HORN_BC_D = 14.0      # datasheet Ø14 bolt circle
HORN_N = 4            # datasheet 4-M3
HORN_SCREW_CLR = 3.4  # M3 clearance (horn holes are Ø3.2; wheel bolts through)


def _build():
    r = WHEEL_D / 2
    w = WHEEL_W
    p = Cylinder(r, w, align=(Align.CENTER, Align.CENTER, Align.MIN))   # base at z=0

    # O-ring grooves around the rim (Torus tube centred on the OD -> a seated channel)
    zs = [GROOVE_INSET + i * (w - 2 * GROOVE_INSET) / max(1, N_GROOVES - 1) for i in range(N_GROOVES)]
    for z in zs:
        p -= Pos(0, 0, z) * Torus(r, ORING_CS / 2)

    # horn registration pocket (mount face = top, z=w) + through center bore
    p -= Pos(0, 0, w - HORN_RECESS_DEPTH) * Cylinder(
        HORN_RECESS_D / 2, HORN_RECESS_DEPTH + 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p -= Pos(0, 0, -1) * Cylinder(CENTER_BORE / 2, w + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # outer-face counterbore to reach the central retaining screw
    p -= Pos(0, 0, -1) * Cylinder(CENTER_CBORE_D / 2, CENTER_CBORE_DEPTH + 1,
                                  align=(Align.CENTER, Align.CENTER, Align.MIN))

    # horn bolt circle (through the hub)
    bc = HORN_BC_D / 2
    for i in range(HORN_N):
        import math
        a = 2 * math.pi * i / HORN_N
        p -= Pos(bc * math.cos(a), bc * math.sin(a), -1) * Cylinder(
            HORN_SCREW_CLR / 2, w + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return p


part = _build()   # module-level built solid (repo convention: check_parts reads `part`)


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    export_stl(part, "build/base_wheel.stl")
    import trimesh
    m = trimesh.load("build/base_wheel.stl")
    print("base_wheel:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
