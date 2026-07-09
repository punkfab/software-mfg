"""coupling_statics.py — how securely does the tool-changer coupling hold, and how well
does it register? Physics-first (analytic -> validity -> min-of-modes), the same
discipline as sim/mobile_base.py: don't print until the numbers say the joint survives
the worst pose the arm can put it in.

The coupling (parts/_coupling.py) is three stacked jobs, and this model scores each:

  REGISTER  3 balls in 3 vees at FEATURE_R -> exact-constraint pose. Its worth is
            REPEATABILITY (how close the tool returns to the same pose), which sets the
            tool-pose sigma the tracker would otherwise have to measure.
  HOLD AXIAL a central Fidlock barb: a magnet only TRIGGERS it, the mechanical catch
            carries pull-off -> hold is independent of magnet strength, FAILS LOCKED.
  HOLD MOMENT the worst case. A tool cantilevered off the wrist is a PRYING MOMENT that
            tries to gap the coupling open. Two limits:
              * LIFTOFF (soft): the least-loaded vee reaches zero preload -> the joint
                starts to gap -> registration is lost (repeatability gone) though nothing
                broke. This is the coupling's "tip-over": M_liftoff = preload x radius.
              * ULTIMATE (hard): past liftoff the tension side rides the rigid ties.
                Moment capacity = tie force x moment arm -> a tie at the RIM (a lug at
                RIM_R) beats a central barb (arm ~ FEATURE_R). This is HOTDOCK's
                "load through the locking ring at the circumference" — quantified.

The point of the run: for a LIGHT arm the central Fidlock alone may already clear the
service moment with margin, so the rim lock is optional weight. The model says which.

Uncertain retention numbers (magnet pull, printed-catch strength, contact friction,
bayonet-ramp preload) are CALIBRATED + seeded UNANCHORED -> every result reads as a
PREDICTION until a bench pull-test on a real coupling anchors them.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "parts"))
import _coupling as geo  # coupling geometry = single source of truth  # noqa: E402
from calibration import param  # noqa: E402

G = 9.81
SAFETY = 1.5           # required margin on the structural (ultimate) modes
MM = 1e-3

# geometry pulled straight from the CAD (mm -> m)
FEATURE_R = geo.FEATURE_R * MM
RIM_R = geo.RIM_R * MM
N_LUG = geo.N_LUG
BALL_D = geo.BALL_D * MM

# --- calibrated, uncertain, UNANCHORED -> results are PREDICTION until measured ----------
F_MAG = param("coupling_magnet_N", 20.0)         # seating magnet / EPM pull (capture + preload)
F_DRAW = param("coupling_fidlock_draw_N", 15.0)  # Fidlock shuttle spring clamp (adds preload)
C_BARB = param("coupling_barb_hold_N", 200.0)    # printed central catch pull-off capacity
C_LUG = param("coupling_lug_hold_N", 80.0)       # ONE printed rim lug tension/bearing capacity
F_LUGPRE = param("coupling_bayonet_preload_N", 30.0)  # total bayonet-ramp clamp (all lugs)
C_POST_SHEAR = param("coupling_post_shear_N", 120.0)  # central post shank in shear (in-plane)
MU = param("coupling_contact_mu", 0.3)           # ball/vee + face friction
CONTACT_K = param("coupling_contact_N_per_um", 3.0)   # joint normal stiffness, N per micron


@dataclass
class Retention:
    """A retention configuration to score. Turn features on/off to compare."""
    name: str
    fidlock: bool = True     # central Fidlock barb (draw preload + structural axial tie)
    rim_lock: bool = True    # rim lugs (bayonet preload + structural moment tie at RIM_R)

    def preload_N(self):
        """Total clamp holding the faces together (reacted at the vee ring)."""
        p = F_MAG
        if self.fidlock:
            p += F_DRAW
        if self.rim_lock:
            p += F_LUGPRE
        return p

    def axial_hold_N(self):
        """Pull-off capacity. The MAGNET is not counted (it only triggers) — hold rides the
        mechanical catches, so it fails locked. Bare magnet config holds by magnet alone."""
        h = 0.0
        if self.fidlock:
            h += C_BARB
        if self.rim_lock:
            h += N_LUG * C_LUG
        return h if h > 0 else F_MAG          # no catch -> a plain magnet coupler (weak)

    def m_liftoff_Nm(self):
        """Soft limit: applied moment that unloads the far vee -> the joint gaps and loses
        registration. = preload reacted at the vee ring (+ bayonet preload at the rim)."""
        m = self.preload_N() * FEATURE_R
        if self.rim_lock:
            m += F_LUGPRE * (RIM_R - FEATURE_R)   # rim preload adds restoring at the larger arm
        return m

    def m_ultimate_Nm(self):
        """Hard limit: past liftoff the tension side rides the rigid ties. A central barb
        reacts at arm ~ FEATURE_R (pivot at the near vee); rim lugs react at ~FEATURE_R+RIM_R
        -> the rim tie's longer arm is the whole point of moving load to the circumference."""
        m = 0.0
        if self.fidlock:
            m += C_BARB * FEATURE_R
        if self.rim_lock:
            m += N_LUG / 2 * C_LUG * (FEATURE_R + RIM_R)   # ~half the lugs in tension
        return m if m > 0 else F_MAG * FEATURE_R           # magnet-only: gap = failure

    def shear_hold_N(self):
        """In-plane force capacity: post shank shear + friction on the preload + rim bearing."""
        h = MU * self.preload_N()
        if self.fidlock:
            h += C_POST_SHEAR
        if self.rim_lock:
            h += N_LUG * C_LUG            # lugs bear in-plane too
        return h

    def spin_hold_Nm(self):
        """Anti-spin about the axis — the kinematic coupling's job. The 3 vee side-walls take
        the tangential ball load; capacity ~ friction on the preload at the vee radius."""
        return MU * self.preload_N() * FEATURE_R + (N_LUG * C_LUG * RIM_R if self.rim_lock else 0)


@dataclass
class ServiceLoad:
    """What the arm does to the coupling at the worst pose. Coupling axis = z (normal to the
    faces). A tool cantilevered off the wrist is the prying-moment case that binds."""
    tool_mass_kg: float          # tool + payload hanging off the coupling
    cg_offset_m: float           # tool CG distance from the coupling face (lever for weight)
    axis_horizontal: bool        # True: tool sticks out sideways -> weight makes a moment
    proc_force_N: float          # process side-force (glue press, nudge) at the tool
    proc_lever_m: float          # its distance from the coupling face

    def axial_N(self):
        """Force trying to separate the faces (along z). Tool hanging straight down loads the
        catch axially; sideways, gravity is mostly shear, not axial."""
        w = self.tool_mass_kg * G
        return (self.proc_force_N if self.axis_horizontal else w + self.proc_force_N)

    def moment_Nm(self):
        """Prying moment about the coupling rim. Weight arm only counts when the axis is
        horizontal (tool cantilevered); the process force always levers on the face."""
        w = self.tool_mass_kg * G
        m = self.proc_force_N * self.proc_lever_m
        if self.axis_horizontal:
            m += w * self.cg_offset_m
        return m

    def shear_N(self):
        """In-plane force: gravity is shear when the axis is horizontal."""
        w = self.tool_mass_kg * G
        return (w if self.axis_horizontal else 0.0) + (0.0 if self.axis_horizontal else 0.0)

    def spin_Nm(self):
        return 0.0    # no deliberate spin torque in these tasks; kept for completeness


def repeatability_um(ret: Retention):
    """Rough kinematic-coupling repeatability estimate: higher preload -> stiffer contacts ->
    the tool returns closer to the same pose. Steel-on-steel reaches ~1um; printed/steel is
    coarser. = a nominal seat error scaled by how hard we clamp. This IS the tool-pose sigma
    the tracker would otherwise measure — flag as PREDICTION (CONCEPT.md §5.2)."""
    base_um = 30.0                      # steel ball into a printed vee, lightly seated
    ref_N = 35.0                        # reference preload the base figure assumes
    return base_um * (ref_N / max(1.0, ret.preload_N())) ** 0.5


def evaluate(ret: Retention, load: ServiceLoad):
    """Score one retention config against one service load. Margins are dimensionless so the
    binding mode is comparable across force/moment (min-of-modes, cf. mobile_base)."""
    demand = {
        "axial": (load.axial_N(), ret.axial_hold_N()),
        "moment_liftoff": (load.moment_Nm(), ret.m_liftoff_Nm()),   # registration-preserving
        "moment_ultimate": (load.moment_Nm(), ret.m_ultimate_Nm()),  # structural
        "shear": (load.shear_N(), ret.shear_hold_N()),
        "spin": (load.spin_Nm(), ret.spin_hold_Nm()),
    }
    margins = {k: (cap / d if d > 1e-9 else float("inf")) for k, (d, cap) in demand.items()}
    binding = min(margins, key=margins.get)
    # PASS: never gap (liftoff margin >= 1) AND clear the structural modes with SAFETY
    struct = ["axial", "moment_ultimate", "shear", "spin"]
    ok = margins["moment_liftoff"] >= 1.0 and all(margins[k] >= SAFETY for k in struct)
    return {
        "name": ret.name,
        "preload_N": round(ret.preload_N(), 1),
        "caps": {k: round(cap, 3) for k, (_, cap) in demand.items()},
        "margins": {k: round(v, 2) for k, v in margins.items()},
        "binding": binding, "binding_margin": round(margins[binding], 2),
        "repeatability_um": round(repeatability_um(ret), 1),
        "pass": ok,
    }


def validity(ret: Retention, load: ServiceLoad):
    w = []
    if ret.axial_hold_N() <= F_MAG * 1.01 and not ret.fidlock:
        w.append("magnet-only hold: drops the tool on power loss / overload — no mechanical catch")
    if evaluate(ret, load)["margins"]["moment_liftoff"] < 1.0:
        w.append("service moment GAPS the joint open — registration lost (add preload or rim lock)")
    if repeatability_um(ret) > 50:
        w.append(f"repeatability ~{repeatability_um(ret):.0f}um is coarse — raise preload or use "
                 "steel vees")
    if not (0.15 <= MU <= 0.6):
        w.append(f"contact friction mu={MU} out of plausible range")
    return w


# ---- the near-term scenario + the three configs to compare -------------------------------
# SO-101 wrist holding a light process tool cantilevered sideways (the worst prying pose):
# a ~0.15 kg glue/shear tool whose CG is 40mm off the face, plus a 5N process nudge at 60mm.
PRINTER_TOOL = ServiceLoad(tool_mass_kg=0.15, cg_offset_m=0.040, axis_horizontal=True,
                           proc_force_N=5.0, proc_lever_m=0.060)

CONFIGS = [
    Retention("magnets-only", fidlock=False, rim_lock=False),   # naive magnetic coupler
    Retention("kinematic + Fidlock (repo)", fidlock=True, rim_lock=False),
    Retention("+ rim lock (HOTDOCK)", fidlock=True, rim_lock=True),
]


if __name__ == "__main__":
    L = PRINTER_TOOL
    print(f"scenario: SO-101 holding a {L.tool_mass_kg}kg tool cantilevered sideways, CG "
          f"{L.cg_offset_m*1000:.0f}mm off the face + {L.proc_force_N}N process force at "
          f"{L.proc_lever_m*1000:.0f}mm")
    print(f"  -> service: prying moment {L.moment_Nm()*1000:.0f} Nmm, axial {L.axial_N():.1f} N, "
          f"shear {L.shear_N():.1f} N   (coupling vees at r={FEATURE_R*1000:.0f}mm, "
          f"rim lugs at r={RIM_R*1000:.0f}mm)\n")
    for ret in CONFIGS:
        r = evaluate(ret, L)
        c = r["caps"]; m = r["margins"]
        print(f"[{r['name']}]  preload {r['preload_N']}N   repeatability ~{r['repeatability_um']}um")
        print(f"   axial hold {c['axial']:.0f}N (x{m['axial']:.1f})   "
              f"moment: liftoff {c['moment_liftoff']*1000:.0f}Nmm (x{m['moment_liftoff']:.1f}) / "
              f"ultimate {c['moment_ultimate']*1000:.0f}Nmm (x{m['moment_ultimate']:.1f})")
        print(f"   shear {c['shear']:.0f}N (x{m['shear']:.1f})   binds: {r['binding']} "
              f"(x{r['binding_margin']})")
        for wv in validity(ret, L):
            print(f"   ! {wv}")
        print(f"   => {'PASS' if r['pass'] else 'FAIL'}  [PREDICTION until a bench pull-test "
              f"anchors magnet/catch/friction]\n")
