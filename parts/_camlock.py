"""_camlock.py — shared parameters for the SERVO-DRIVEN CENTRAL DRAW-LOCK coupling.

An alternative to the Fidlock changer (parts/_coupling.py). Same idea for REGISTRATION —
a 3-point kinematic locator — but here at the classic EQUAL 120deg (fully symmetric, no
keying) and the retention is a central cam the arm actuates with a servo:

    a POST on the tool carries 3 interrupted ACME-thread lugs (a "breech" thread);
    a CAM RING in the arm carries the matching interrupted internal thread + gaps.
    Drop the post in through the gaps -> the servo turns the cam ~a quarter turn -> the
    helical thread catches the lugs and DRAWS the post axially into the kinematic seat,
    generating a commandable preload. A shallow lead (< the friction angle) is SELF-
    LOCKING, so it holds with the servo unpowered (fails locked, no magnet needed).

Why this over the magnet-triggered Fidlock:
  + actively seats + sets its OWN preload (servo torque -> preload via the screw ratio),
    so registration is driven home every time (not left to a magnet's last-mm pull);
  + huge mechanical advantage: a 1 N.m servo makes 100s of N of draw through the lead;
  + self-locking thread holds unpowered.
  - one moving cam + a servo per changer, and the cam's locking torque reacts on the
    kinematic seat (the 120deg vees must hold it — geometric anti-spin; see statics).

Shares the disc / ball / vee geometry with _coupling so tools stay interchangeable.
sim/camlock_statics.py scores the screw draw, self-lock, and the reaction-vs-seat check.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import (Align, Axis, Box, BuildPart, BuildSketch, Cylinder, Locations,  # noqa: E402
                       Plane, Pos, Rectangle, Rot, revolve)

import _coupling as base  # noqa: E402  # shared disc + ball + vee geometry

C, MN, MX = Align.CENTER, Align.MIN, Align.MAX

# --- disc + mount (bigger than the Fidlock: the central cam ring is ~Ø30, so the vees +
#     bolts live outside it -> a Ø64 plate. A different, larger, actively-driven design pt) ---
PLATE_OD = 64.0
PLATE_T = 6.0
BOLT_R = 28.5
BOLT_CLR = 3.4
MOUNT_ANGLES = (30.0, 150.0, 270.0)
BALL_D = base.BALL_D

# --- 1. registration: 3-point kinematic locator at EQUAL 120deg (symmetric, no key) ---
FEATURE_R = 26.0
FEATURE_ANGLES = (90.0, 210.0, 330.0)     # equal 120deg -> self-centring, spin-constrained
BALL_POCKET_D = BALL_D + 0.4
BALL_POCKET_DEPTH = 5.0
VEE_DEPTH = 3.0
VEE_W = 8.0
VEE_L = 10.0                              # short -> stays outside the central cam bore

# --- 2. central draw-lock: interrupted ACME thread (post lugs + cam catches) ---
POST_SHAFT_D = 14.0        # tool post shaft Ø (mean thread Ø) -> r7
POST_H = 13.0              # post height above the tool face (enters the arm + cam)
N_START = 3               # interrupted thread starts / lugs (120deg apart)
LUG_ANGLES = (30.0, 150.0, 270.0)   # between the balls; align with the cam GAPS to insert
R_LUG = 10.5              # lug outer radius (projects 3.5 past the shaft)
LUG_ARC = 40.0           # each lug's angular width (deg)
LUG_T = 3.0              # lug axial thickness
LUG_Z = 8.5              # lug underside height above the tool face (cam hooks under here)

LEAD = 8.0               # thread lead: axial draw per FULL turn (mm). Sets self-lock + draw.
ENGAGE_ANGLE = 70.0      # cam rotation from insert to full lock (~a quarter turn)
DRAW = LEAD * ENGAGE_ANGLE / 360.0       # axial draw over the lock rotation (~1.6mm)

R_CATCH_IN = 8.5         # cam catch inner radius (grabs the lug: r8.5..10.5 = 2mm engage)
CATCH_ANGLES = (90.0, 210.0, 330.0)   # catches sit between the gaps (gaps at the LUG_ANGLES)
CATCH_ARC = 70.0         # each cam catch arc; gaps = 120 - CATCH_ARC = 50deg (> LUG_ARC -> pass)
GAP_BORE_R = R_LUG + 1.0  # bore radius at the gaps so the lugs pass axially
CAM_OD = 30.0            # cam ring body Ø that rotates in the arm bore -> r15
CAM_H = 9.0              # cam ring body height
CAM_FLANGE_OD = 36.0     # top flange -> bears on the arm plate top face (takes draw reaction)
CAM_FLANGE_T = 3.0
CATCH_H = 3.5            # catch axial thickness (ramped top draws the lug up)
ARM_BORE_D = CAM_OD + 1.0  # arm central bore the cam body rotates in

# --- 3. servo drive: the cam bore is open for the post, so the ring is driven from a SIDE
#     DRIVE ARM (a radial tab). A servo horn + short link (or a pinion on a ring-gear) swings
#     it ~ENGAGE_ANGLE. Keeps the centre clear; the servo mounts off to the side on the arm. ---
DRIVE_ARM_ANGLE = 0.0
DRIVE_ARM_LEN = 11.0     # tab reach past the flange edge
DRIVE_ARM_W = 8.0
DRIVE_PIN_D = 3.2        # link pin hole at the tab tip
BRACKET_SCREW_CLR = 2.7  # M2.5 clearance for the servo-bracket mount holes

# --- 4. power/data: one pogo cluster in the annulus between the cam bore and the vees ---
POGO_ANGLE = 145.0
POGO_R = 18.0
POGO_N = 4
POGO_PITCH = 3.0
POGO_BORE_D = 2.2

pos_at = base.pos_at


def plate_blank(bore=0.0):
    """Ø PLATE_OD disc + 3 M3 mount holes (+ optional central bore). base z=0, top PLATE_T."""
    p = Cylinder(PLATE_OD / 2, PLATE_T, align=(C, C, MN))
    if bore:
        p -= Pos(0, 0, -1) * Cylinder(bore / 2, PLATE_T + 2, align=(C, C, MN))
    for a in MOUNT_ANGLES:
        x, y = pos_at(BOLT_R, a)
        p -= Pos(x, y, -1) * Cylinder(BOLT_CLR / 2, PLATE_T + 2, align=(C, C, MN))
    return p


def pogo_bores(z_from, depth):
    """A row of POGO_N pin bores at POGO_ANGLE, centred on POGO_R (tangential row)."""
    cx, cy = pos_at(POGO_R, POGO_ANGLE)
    tang = (-math.sin(math.radians(POGO_ANGLE)), math.cos(math.radians(POGO_ANGLE)))
    out = []
    for i in range(POGO_N):
        off = (i - (POGO_N - 1) / 2) * POGO_PITCH
        x, y = cx + tang[0] * off, cy + tang[1] * off
        out.append(Pos(x, y, z_from) * Cylinder(POGO_BORE_D / 2, depth, align=(C, C, MN)))
    return out


def annular_sector(r_in, r_out, z0, h, a_center, arc):
    """A watertight annular sector (pie slice of a ring), centred on angle `a_center`.
    Built by revolving a radial rectangle about Z through `arc` degrees (revolve makes a
    clean single solid — no sphere/boolean artifacts)."""
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):
            with Locations(((r_in + r_out) / 2, z0 + h / 2)):
                Rectangle(r_out - r_in, h)
        revolve(axis=Axis.Z, revolution_arc=arc)
    return Rot(0, 0, a_center - arc / 2) * p.part


def ramped_catch(r_in, r_out, z0, h, a_center, arc, rise):
    """An annular-sector catch whose TOP face ramps up by `rise` across its arc (a helical
    thread segment). The lug's underside rides this ramp as the cam turns -> axial draw.
    The ramp is a tilted-plane cut about the radial axis (linear ~= helical over a small
    arc); trapezoidal ACME flanks are a machining refinement, noted, not printed."""
    seg = annular_sector(r_in, r_out, z0, h + rise + 1, a_center, arc)
    rm = (r_in + r_out) / 2
    tang = rm * math.radians(arc)                 # tangential length of the segment
    tilt = math.degrees(math.atan2(rise, tang))   # ramp angle
    # cut a big tilted box down onto the segment, hinged about the radial axis at a_center
    cutter = Rot(0, 0, a_center) * Rot(tilt, 0, 0) * Pos(0, 0, z0 + h) \
        * Box(4 * r_out, 4 * r_out, 20, align=(C, C, MN))
    return seg - cutter
