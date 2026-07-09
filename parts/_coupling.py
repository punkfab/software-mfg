"""_coupling.py — shared parameters for the kinematic tool-changer coupling.

A Maxwell coupling (3 balls in 3 vee-grooves) gives exact-constraint, self-centering
seating (CONCEPT.md §5.2). The arm-side carries the vees, the tool-side carries the
balls. Both share this blank: a disc with a wiring bore and a 3-bolt mount pattern.

This is the SIM-SIDE design study. The canonical latch geometry is read-only in
`../robot-effectors/.../toolchanger-fidlock-kinematic/coupling_latch.py`; we compose
that by reference and mirror its good ideas here so the sim + statics stay honest.

Three ideas, in order of how much load they carry (see sim/coupling_statics.py):

  1. REGISTRATION — 3 balls/vees at FEATURE_R, spaced UNEQUALLY so the tool seats in
     exactly ONE orientation (keys the pogo set + jaw direction — no duplicate pins).
  2. AXIAL HOLD  — a central Fidlock catch: a permanent magnet only TRIGGERS a
     mechanical barb; the barb carries the pull-off load, so hold is independent of
     magnet strength and FAILS LOCKED (zero standing power). Modelled here as the
     mushroom POST (tool) + head-pocket bore (arm); the moving keyhole shuttle lives
     in the canonical part (our sim abstracts it as an EPM-weld toggle).
  3. MOMENT HOLD — a RIM LOCK: lugs out at RIM_R hand the prying moment to the
     circumference (HOTDOCK's "locking ring"). Moment capacity scales with radius, so
     a rim tie beats a central one for the same preload. Optional; statics says if the
     central catch alone already covers the service moment for a light arm.

`_`-prefixed so scripts/check_parts.py skips it; the two halves import it.
"""

import math

from build123d import Align, Box, Cylinder, Pos, Rot

C, MN = Align.CENTER, Align.MIN

# --- disc + mount (interface contract shared with the glue/shear tool bodies) ---
PLATE_OD = 50.0      # coupling disc outer Ø
PLATE_T = 6.0        # disc thickness
BORE_D = 12.0        # tool-MOUNTING interchange bore (glue/shear bodies mate this — keep it)
WIRE_BORE_D = 6.0    # the coupling FACE's own small wiring bore (latch pin / signal pass-through)
BOLT_R = 21.0        # mounting bolt circle
BOLT_CLR = 3.4       # M3 clearance
MOUNT_ANGLES = (30, 150, 270)     # 3 mounting bolts

# --- 1. registration: 3 vees/balls, UNEQUAL spacing -> single orientation (keyed) ---
FEATURE_R = 16.0                  # pitch radius of the 3 coupling features
FEATURE_ANGLES = (80, 210, 330)   # 130/120/110 deg apart: breaks 3-fold symmetry ->
#                                   the tool drops in exactly ONE clocking. Kept >=50deg
#                                   off every MOUNT_ANGLE so vees never cross a bolt hole.
BALL_D = 8.0                      # coupling ball Ø — a BOUGHT steel ball (Ø8 chrome/G25),
#                                   glued into a printed pocket (hardened + repeatable, and
#                                   no printed sphere = no pole-facet STL artifact)
BALL_POCKET_D = BALL_D + 0.4      # pocket bore (slip + glue)
BALL_POCKET_DEPTH = 5.0           # pocket depth -> ball protrudes BALL_D - DEPTH = 3mm
VEE_DEPTH = 3.0                   # groove depth into the arm-side top face
VEE_W = 8.0                       # 45deg-diamond cutter section -> a 90deg vee
VEE_L = 12.0                      # radial groove length (short -> stays inside the rim ring)

# --- 2. axial hold: central Fidlock catch (mushroom post + head pocket) ---
LATCH_NECK_D = 6.0                # post neck Ø
LATCH_HEAD_D = 10.0               # post head (barb) Ø — the under-face is the catch
LATCH_NECK_H = 5.0               # neck height above the tool face
LATCH_HEAD_H = 3.0               # head height
LATCH_PIN_D = 3.1                 # axial steel pin bore (the magnet target)
POCKET_CLR = 0.6                  # arm-side pocket clearance over the head/neck

# --- 3. moment hold: optional RIM LOCK (lugs at the circumference) ---
RIM_LOCK = True                   # include the rim lugs/catch (statics evaluates its worth)
RIM_R = 21.5                      # lug pitch radius (near the OD, inside PLATE_OD/2=25)
N_LUG = 3
LUG_ANGLES = tuple(FEATURE_ANGLES[i] + 60 for i in range(N_LUG))  # interleave with the vees
LUG_W = 7.0                       # lug tangential width
LUG_T = 3.0                       # lug thickness (axial) — bears against the arm catch lip
LUG_SPAN = 8.0                    # lug radial length (centred on RIM_R -> projects past OD)
CATCH_H = LUG_T + 1.0             # arm-side catch-ring height the lug hooks under
CATCH_RING_IR = 22.0             # catch ring inner radius (clears the vees at r<=22)
CATCH_RING_OR = 25.0             # catch ring outer radius (= disc OD/2)

# --- power/data: ONE keyed pogo cluster (single orientation from the vees -> no duplicate) ---
POGO_ANGLE = 145.0                # sits in the gap between two vees, clear of the latch
POGO_R = 10.5                     # radius of the pad cluster (annulus between latch & vees)
POGO_N = 4                        # pins in the cluster (power+, power-, data, gnd)
POGO_PITCH = 3.0                  # spacing of the pin bores
POGO_BORE_D = 2.2                 # pin/pad bore Ø


def pos_at(r, angle_deg):
    return r * math.cos(math.radians(angle_deg)), r * math.sin(math.radians(angle_deg))


# back-compat name used by the two halves / glue_check
FEATURE_pos = pos_at


def plate_blank(bore=WIRE_BORE_D):
    """Disc + 3 M3 mount holes + a small central wiring bore. base z=0, top z=PLATE_T."""
    p = Cylinder(PLATE_OD / 2, PLATE_T, align=(C, C, MN))
    if bore:
        p -= Pos(0, 0, -1) * Cylinder(bore / 2, PLATE_T + 2, align=(C, C, MN))
    for a in MOUNT_ANGLES:
        x, y = pos_at(BOLT_R, a)
        p -= Pos(x, y, -1) * Cylinder(BOLT_CLR / 2, PLATE_T + 2, align=(C, C, MN))
    return p


def pogo_bores(z_from, depth):
    """A row of POGO_N pin bores at POGO_ANGLE, as a list of solids to cut (or a boss to
    add). Centred on POGO_R; the whole cluster is off-axis so it keys with the vees."""
    cx, cy = pos_at(POGO_R, POGO_ANGLE)
    tang = (-math.sin(math.radians(POGO_ANGLE)), math.cos(math.radians(POGO_ANGLE)))
    bores = []
    for i in range(POGO_N):
        off = (i - (POGO_N - 1) / 2) * POGO_PITCH
        x, y = cx + tang[0] * off, cy + tang[1] * off
        bores.append(Pos(x, y, z_from) * Cylinder(POGO_BORE_D / 2, depth, align=(C, C, MN)))
    return bores
