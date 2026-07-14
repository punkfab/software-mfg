"""_acme_changer.py — shared geometry + the multi-start ACME thread for the servoless tool changer.

Numbers come straight from the validated statics (sim/toolchanger_acme.py): Ø38 mean, 4-start, 8 mm
lead, self-locking, 25 N preload from the TPU vees. The MALE thread is modelled once (helix sweep);
the FEMALE ring bore is cut with the SAME male grown by a print clearance, so they can't mis-mate.

    from _acme_changer import acme_male, RING, RECV, ...   # used by parts/acme_*.py
"""
import math

from build123d import (Align, Axis, BuildLine, BuildPart, BuildSketch, Cylinder, Helix, Plane, Pos,
                       Polygon, Rot, sweep)

# --- ACME thread (from the sim) -------------------------------------------------------------
MEAN_D = 38.0                       # mean Ø
DEPTH = 2.0                         # radial thread depth
MAJOR_D = MEAN_D + DEPTH            # 40  (male crest / female clearance Ø)
MINOR_D = MEAN_D - DEPTH            # 36  (male root)
STARTS = 4                          # 4-start "Gatorade" -> engages within 90° of any orientation
PITCH = 2.0                         # crest-to-crest (axial)
LEAD = STARTS * PITCH               # 8 mm/turn
FLANK_DEG = 14.5                    # ACME flank half-angle
THREAD_LEN = 12.0                  # engaged thread length
FIT_CLR = 0.35                      # radial clearance cut into the female (printed slip fit)


def _profile_pts(major_d, minor_d, pitch, flank_deg, weld=1.0):
    """ACME trapezoid in the sweep plane's local (u=radial-out, v=axial) coords, centred on the mean
    radius. Crest (outer) = the narrow flat; root (inner) is wider by the flanks and is sunk `weld`
    mm BELOW the minor Ø so the ridge fuses into the core with real volume overlap (not a sliver)."""
    depth = (major_d - minor_d) / 2.0
    mean_r = (major_d + minor_d) / 4.0
    crest = 0.37 * pitch                                  # crest flat
    root = crest + 2 * depth * math.tan(math.radians(flank_deg))
    u0 = minor_d / 2.0 - mean_r - weld                    # root, sunk `weld` into the core
    u1 = major_d / 2.0 - mean_r                           # crest radius, relative to mean
    return [(u0, -root / 2), (u1, -crest / 2), (u1, crest / 2), (u0, root / 2)], mean_r


def acme_male(length=THREAD_LEN, major_d=MAJOR_D, minor_d=MINOR_D, lead=LEAD, starts=STARTS,
              flank_deg=FLANK_DEG):
    """Solid external ACME thread: a minor-Ø core + `starts` helical ridges. Watertight single body."""
    from build123d import Box
    pitch = lead                                          # each start advances LEAD per full turn
    pts, mean_r = _profile_pts(major_d, minor_d, lead / starts, flank_deg)
    over = 2 * pitch
    solid = Pos(0, 0, -over) * Cylinder(minor_d / 2, length + 2 * over,
                                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    with BuildPart() as ridge:                            # one ridge along the helix
        with BuildLine():
            h = Helix(pitch=lead, height=length + 2 * over, radius=mean_r)
        with BuildSketch(Plane(origin=h @ 0, x_dir=(1, 0, 0), z_dir=h % 0)):   # local +x = RADIAL
            Polygon(*pts, align=None)
        sweep(is_frenet=True)
    r = Pos(0, 0, -over) * ridge.part
    for k in range(starts):                               # rotate into the other starts
        solid += Rot(0, 0, 360.0 * k / starts) * r
    big = 3 * major_d                                      # trim the over-run to a clean [0, length]
    return solid & (Pos(0, 0, length / 2) * Box(big, big, length))


def acme_female_cutter(length=THREAD_LEN, clearance=FIT_CLR, **kw):
    """The negative to cut a mating internal thread: the male grown radially by `clearance`."""
    return acme_male(length=length, major_d=MAJOR_D + 2 * clearance,
                     minor_d=MINOR_D + 2 * clearance, **kw)


# --- shared interface dims ------------------------------------------------------------------
HORN_BC_D = 14.0        # STS3215 wrist-roll horn bolt circle (Ø14, 4×M3) — arm piece bolts here
HORN_N = 4
M3_CLR = 3.4
RING_AF = 46.0         # hex ring across-flats (dock wrench drives this); corners at r=26.6
BALL_D = 8.0           # bought steel locator balls
BALL_PCD = 60.0        # 3 @120°, OUTBOARD of the ring so they reach the arm's vee plate (r30 > 26.6)
VEE_ANGLE = 90.0       # TPU vee included angle
TOOL_BORE_D = 12.0     # tool-mount interchange bore (matches coupling_tool_side contract)
TOOL_BOLT_R = 21.0     # 3×M3 tool-mount bolt circle
TOOL_BOLT_ANG = (30, 150, 270)
