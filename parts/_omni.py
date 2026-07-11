"""_omni.py — shared parametric geometry for the reverse-engineered omni wheel.

Grey-box reverse engineering (not tracing): the STRUCTURE is known — an omni wheel is barrel
rollers on tangent axes, each roller's outer surface lying on the wheel's pitch circle
(radius R_EFF) so the barrels blend into a round OD. MEASURE your wheel and set R_EFF /
N_ROLLERS / MOUNT_R; the rest derives. Key identities:

    rho(z) = R_EFF - hypot(MOUNT_R, z)      # barrel radius keeps the surface on the OD circle
    HALF_L = MOUNT_R * tan(pi/N) - gap/2    # roller half-length (with a gap for the hub)

**Continuity fix:** a single row of rollers with gaps drops into a gap each turn — it bumps,
it doesn't roll. So the wheel is TWO staggered rows (a "double omni wheel"): row 2 is rotated
half a pitch, so its rollers sit over row 1's gaps. Contact is continuous iff each roller's
angular coverage half-angle >= 90/N degrees (see continuity()).
"""
import math

R_EFF = 30.0        # wheel effective radius (mm) -> 60 mm OD.        MEASURE
N_ROLLERS = 5       # rollers PER ROW — count them on the real wheel. MEASURE
MOUNT_R = 22.0      # roller-axis distance from wheel centre (pitch). MEASURE
PIN_D = 3.0         # axle pin (3 mm dowel / M3)
ROLLER_BORE = 3.4   # roller spins on the pin -> bore > pin
HUB_PIN_BORE = 3.2  # round seat the pin snaps INTO (holds the pin; the roller spins on it)
PIN_SNAP_MOUTH = 2.4  # radial entry throat, NARROWER than the pin -> lips flex, pin snaps past &
#                       is retained. ~0.5mm snap interference (throat < PIN_D); inboard seat = the nest.
ROLLER_SAMPLES = 28
ROLLER_GAP_MM = 10.0  # tangential gap between rollers in a row -> room for the hub + spin clearance
# Optional MEASURED roller half-length: some wheels' barrels overhang the axle pins, so the length
# isn't the "rollers meet at the pitch angle" identity. Set to a number (mm) to override the derived
# value; None uses FULL_HALF_L - ROLLER_GAP_MM/2. (The kiwi-v10 wheel's barrels overhang ~2.6x its
# pin span, giving ~±28deg coverage — see scripts/omni_check.py's geometric continuity check.)
ROLLER_HALF_L = None
ROWS = 2              # two axially-offset, half-pitch-staggered rows -> continuous contact

# drive interface to the STS3215 horn (same approach as parts/base_wheel.py) — MEASURE horn
HUB_BORE = 8.5
HORN_BC_D = 16.0
HORN_N = 4
HORN_SCREW = 2.7

FULL_HALF_L = MOUNT_R * math.tan(math.pi / N_ROLLERS)     # where rollers would just meet
# derived length (rollers nearly meet, minus a hub gap) unless a measured length overrides it
HALF_L = ROLLER_HALF_L if ROLLER_HALF_L is not None else FULL_HALF_L - ROLLER_GAP_MM / 2
BARREL_MAX = R_EFF - MOUNT_R                              # barrel radius at its centre
END_R = R_EFF - math.hypot(MOUNT_R, HALF_L)              # barrel radius at the (shortened) ends
ROW_STAGGER = 180.0 / N_ROLLERS                          # deg: row-2 rotated half a pitch
ROW_Z = BARREL_MAX + 1.0                                 # axial half-separation of the two rows


def rho(z):
    """Barrel radius at axial station z, holding the outer surface on the wheel pitch circle."""
    return R_EFF - math.hypot(MOUNT_R, z)


def roller_center(i, row=0):
    """((x, y, z), angle_deg) of roller i in `row`. Row 1 is staggered half a pitch and offset
    axially so its rollers cover row 0's gaps."""
    a = 2 * math.pi * i / N_ROLLERS + (math.radians(ROW_STAGGER) if row else 0.0)
    z = ROW_Z if row else -ROW_Z
    return (MOUNT_R * math.cos(a), MOUNT_R * math.sin(a), z), math.degrees(a)


def coverage_half_deg():
    """Half the azimuth a single roller keeps at the OD, ESTIMATED from the axle-pin span
    (surface on R_EFF out to ±HALF_L). This is a LOWER BOUND: barrels that overhang their pins
    cover more (the kiwi-v10 wheel covers ~±28deg vs ~±11 from its pin span). The authoritative
    continuity test measures the ASSEMBLED geometry — scripts/omni_check.py:geometric_continuity."""
    return math.degrees(math.atan(HALF_L / MOUNT_R))


def continuity():
    """ANALYTIC ESTIMATE (advisory): two staggered rows -> combined rollers every 180/N deg, each
    covering ±coverage; contact is continuous iff coverage >= 90/N (adjacent arcs meet). Because
    coverage_half_deg() is a lower bound, this can FALSE-NEGATIVE for barrels that overhang the
    pins — the gate uses the geometric OD-coverage measurement, not this."""
    need = 90.0 / N_ROLLERS
    cov = coverage_half_deg()
    return {"coverage_deg": round(cov, 1), "need_deg": round(need, 1),
            "continuous": cov >= need, "margin": round(cov / need, 2)}


def validity():
    w = []
    if R_EFF <= MOUNT_R:
        w.append("R_EFF must exceed MOUNT_R")
    if END_R <= ROLLER_BORE / 2 + 1.0:
        w.append("barrel ends too thin for the bore (raise R_EFF or MOUNT_R)")
    if BARREL_MAX <= 2:
        w.append("barrel too shallow (rollers won't contact ground)")
    # NB: contact continuity is validated GEOMETRICALLY on the assembled wheel
    # (scripts/omni_check.py:geometric_continuity), not by the analytic continuity() lower bound.
    return w
