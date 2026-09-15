"""_snapfit.py — shared geometry for the snap-fit peg-in-socket: the first autonomous assembly.

Two printed parts fastened by DESIGNED geometry (no separate fastener, no feeder): a PEG part with a
tapered lead-in nose + an annular snap bead, and a SOCKET part whose mouth chamfer is the CAPTURE CONE
(the affordance we sweep) and whose internal groove catches the bead; relief slots let the mouth flex
to admit the bead. Manufacturing-for-Design in the small: the chamfer widens the capture radius past
the base's pose scatter, so a cheap, imprecise arm still seats the part. Both parts share this module so
the mating dims can never drift.

    from _snapfit import PEG_D, BORE_D, CHAMFER_DEPTH, peg_profile, ...   # used by parts/snap_*.py
"""

# --- mating interface (peg <-> socket) ------------------------------------------------------
PEG_D      = 12.0      # peg shaft Ø
CLEARANCE  = 0.4       # bore - shaft, printed slip fit
BORE_D     = PEG_D + CLEARANCE          # 12.4  socket bore
PEG_LEN    = 22.0      # peg length above its shoulder
NOSE_LEN   = 4.0       # tapered lead-in at the peg tip (finds the socket chamfer)
NOSE_R     = 3.0       # peg tip radius at the very end
BEAD_D     = 13.2      # snap-bead OD -> 0.6 mm radial interference over the shaft
BEAD_W     = 2.4       # bead axial width (ramped both sides -> cams in AND out)
BEAD_POS   = 16.0      # bead centre, from the peg shoulder toward the tip (6 mm below the tip)

# --- the AFFORDANCE we sweep in the sim ------------------------------------------------------
CHAMFER_DEPTH = 3.0    # socket mouth lead-in depth; capture radius ~= CHAMFER_DEPTH at 45deg. SWEEP THIS.

# --- socket body ----------------------------------------------------------------------------
GROOVE_D   = BEAD_D + 0.4               # internal detent groove Ø the bead snaps into (13.6)
GROOVE_W   = BEAD_W + 0.6               # groove axial width, a little slop (3.0)
SOCK_H     = PEG_LEN + 2.0             # socket block height (peg bottoms with 2 mm margin)
N_SLOTS    = 3                          # relief slots so the mouth petals flex to admit the bead
SLOT_W     = 1.6                        # slot width
SLOT_REACH = 3.0                        # how far each slot cuts radially into the wall
BLOCK      = 24.0                      # square footprint of each part (gripper-holdable)
BASE_T     = 8.0                       # peg part base-plate thickness

# seated geometry: when the bead sits in the groove, the peg shoulder is ~flush with the socket top.
GROOVE_Z   = SOCK_H - (PEG_LEN - BEAD_POS)   # groove centre height, from socket base (SOCK_H - 6)


def peg_profile():
    """Half-profile (r, z) of the peg, z measured from the shoulder (0) to the tip (PEG_LEN).
    Revolved about Z it is one clean solid: shaft -> ramped bead -> shaft -> tapered nose. The base
    end dips to z=-1 so it welds into the peg part's base plate with real overlap."""
    return [
        (0.0,        -1.0),                       # into the base plate (weld overlap)
        (PEG_D / 2,  -1.0),
        (PEG_D / 2,  BEAD_POS - BEAD_W / 2),      # shaft up to the bead's lower ramp
        (BEAD_D / 2, BEAD_POS),                   # bead crest
        (PEG_D / 2,  BEAD_POS + BEAD_W / 2),      # back down to shaft (upper ramp)
        (PEG_D / 2,  PEG_LEN - NOSE_LEN),         # shaft up to the nose
        (NOSE_R,     PEG_LEN),                    # lead-in taper to the tip
        (0.0,        PEG_LEN),                    # top centre
    ]


def capture_radius(chamfer_depth=CHAMFER_DEPTH):
    """The lateral catch radius the mouth chamfer provides (45deg lead-in): the peg tip is caught and
    funnelled home for any lateral offset up to here. This is the affordance the sim sweeps and the
    number the acceptance gate compares against margin * base pose-sigma."""
    return chamfer_depth + (BORE_D - PEG_D) / 2.0   # chamfer reach + half the slip clearance
