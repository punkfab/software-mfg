"""camlock_statics.py — does the servo-driven central draw-lock hold, seat, and NOT spin
its own tool while it cinches? Physics-first, same min-of-modes discipline as
sim/mobile_base.py and sim/coupling_statics.py, geometry imported from parts/_camlock.py.

The draw-lock is a power screw (interrupted ACME thread) the servo turns to pull the tool
into a 120deg kinematic seat. Four questions decide the design:

  1. SELF-LOCK  — is the lead shallow enough (lead angle < friction angle) that it holds
                  with the servo UNPOWERED? (fails locked, like the Fidlock, but mechanical.)
  2. PRELOAD    — how much axial clamp does the servo make through the screw ratio? (A
                  little servo torque -> a lot of preload; the preload is COMMANDABLE.)
  3. ANTI-SPIN  — the NEW risk this design adds: cinching the screw reacts a torque back on
                  the tool that tries to spin it. The 120deg vees must hold it GEOMETRICALLY.
                  Neat result: capacity and reaction both scale with preload, so the margin
                  is set by geometry alone (R_vee vs the lead) — independent of servo torque.
  4. HOLD       — axial pull-off + prying-moment liftoff, vs the service load (same modes as
                  the Fidlock coupling, but with a higher, commandable preload).

Uncertain numbers (thread + contact friction, servo torque, printed-lug strength) are
CALIBRATED + UNANCHORED -> PREDICTION until a bench test on a real coupling anchors them.
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
sys.path.insert(0, str(_ROOT / "sim"))
import _camlock as geo  # noqa: E402
import coupling_statics as fidlock  # for an apples-to-apples comparison  # noqa: E402
from calibration import param  # noqa: E402

G = 9.81
SAFETY = 1.5
MM = 1e-3

# geometry from the CAD (mm -> m)
DM = geo.POST_SHAFT_D * MM              # mean thread Ø
LEAD = geo.LEAD * MM                    # axial advance per full turn
FEATURE_R = geo.FEATURE_R * MM          # vee pitch radius (kinematic seat + anti-spin arm)
R_LUG = geo.R_LUG * MM                  # thread-lug radius (central moment tie)
N_LUG = geo.N_START
ENGAGE = geo.ENGAGE_ANGLE

# calibrated, uncertain, UNANCHORED
THREAD_MU = param("camlock_thread_mu", 0.25)     # printed ACME thread friction
ACME_HALF = math.radians(14.5)                   # ACME flank half-angle
T_SERVO_CONT = param("camlock_servo_cont_Nm", 1.0)   # STS3215 safe continuous torque
T_SERVO_STALL = param("camlock_servo_stall_Nm", 3.0)  # STS3215 stall (12V)
C_LUG_SHEAR = param("camlock_lug_shear_N", 150.0)    # ONE printed thread-lug shear capacity
VEE_ANTISPIN_K = param("camlock_vee_antispin_k", 1.0)  # vee tangential/normal factor (90deg vee)
CONTACT_MU = param("camlock_contact_mu", 0.3)
TARGET_PRELOAD = param("camlock_target_preload_N", 60.0)  # commanded clamp (good seat, low stress)


def lead_angle():
    return math.atan2(LEAD, math.pi * DM)


def friction_angle():
    """Effective thread friction angle (ACME flank raises it by 1/cos(half-angle))."""
    return math.atan(THREAD_MU / math.cos(ACME_HALF))


def self_locking():
    """Holds unpowered iff lead angle < friction angle. margin = tan(phi)/tan(lambda) (>1 ok)."""
    lam, phi = lead_angle(), friction_angle()
    return {"lead_deg": round(math.degrees(lam), 1), "friction_deg": round(math.degrees(phi), 1),
            "self_locking": lam < phi, "margin": round(math.tan(phi) / math.tan(lam), 2)}


def efficiency():
    """Power-screw efficiency (input torque -> axial work). Self-locking screws are < 0.5."""
    lam, phi = lead_angle(), friction_angle()
    return math.tan(lam) * (1 - math.tan(lam) * math.tan(phi)) / (math.tan(lam) + math.tan(phi))


def preload_from_servo(T):
    """Axial clamp F from servo torque T through the screw: F = 2*pi*eta*T / lead."""
    return 2 * math.pi * efficiency() * T / LEAD


def servo_for_preload(F):
    return F * LEAD / (2 * math.pi * efficiency())


def thread_reaction_torque(F):
    """Torque the thread exerts back on the POST (the anti-spin demand on the vees) to hold a
    clamp F: T = F * dm/2 * tan(lambda + phi). (< the servo torque; the flange thrust friction,
    reacted by the arm, eats the rest.)"""
    return F * DM / 2 * math.tan(lead_angle() + friction_angle())


def antispin_capacity(preload):
    """Torque the 3 120deg vees resist about the axis. A ball in a radial vee resists tangential
    motion by climbing the flanks (geometric), not just friction -> capacity ~ preload * R * k."""
    return VEE_ANTISPIN_K * preload * FEATURE_R


def antispin(preload):
    demand = thread_reaction_torque(preload)
    cap = antispin_capacity(preload)
    # margin is geometry-only: cap/demand = k*R / (dm/2 * tan(lam+phi)) -- independent of preload
    return {"reaction_Nm": round(demand, 3), "capacity_Nm": round(cap, 3),
            "margin": round(cap / demand, 2)}


# --- hold modes (reuse the Fidlock ServiceLoad; add the camlock capacities) ------------------
def axial_hold_N():
    """Pull-off = the thread lugs in shear (a positive mechanical lock, self-holding)."""
    return N_LUG * C_LUG_SHEAR


def m_liftoff_Nm(preload):
    """Soft limit: prying moment that unloads the far vee -> seat gaps -> registration lost."""
    return preload * FEATURE_R


def m_ultimate_Nm():
    """Hard limit: the rigid central post/lugs tie the tension side; arm ~ FEATURE_R (pivot at
    the near vee, tie through the central post)."""
    return N_LUG / 2 * C_LUG_SHEAR * FEATURE_R


def shear_hold_N(preload):
    return CONTACT_MU * preload + N_LUG * C_LUG_SHEAR      # post + lugs bear in-plane


def repeatability_um(preload):
    return 30.0 * (35.0 / max(1.0, preload)) ** 0.5       # same model as the Fidlock coupling


def evaluate(preload, load: "fidlock.ServiceLoad"):
    demand = {
        "axial": (load.axial_N(), axial_hold_N()),
        "moment_liftoff": (load.moment_Nm(), m_liftoff_Nm(preload)),
        "moment_ultimate": (load.moment_Nm(), m_ultimate_Nm()),
        "shear": (load.shear_N(), shear_hold_N(preload)),
    }
    margins = {k: (cap / d if d > 1e-9 else float("inf")) for k, (d, cap) in demand.items()}
    binding = min(margins, key=margins.get)
    ok = margins["moment_liftoff"] >= 1.0 and all(
        margins[k] >= SAFETY for k in ("axial", "moment_ultimate", "shear"))
    return {"preload_N": round(preload, 1), "caps": {k: c for k, (_, c) in demand.items()},
            "margins": {k: round(v, 2) for k, v in margins.items()}, "binding": binding,
            "repeatability_um": round(repeatability_um(preload), 1), "pass": ok}


def validity(preload, load):
    w = []
    sl = self_locking()
    if not sl["self_locking"]:
        w.append(f"NOT self-locking (lead {sl['lead_deg']} >= friction {sl['friction_deg']} deg) "
                 "-> back-drives, needs servo holding torque or a brake; drop the lead")
    elif sl["margin"] < 1.3:
        w.append(f"self-lock margin thin ({sl['margin']}x) — a smaller lead is safer on printed "
                 "plastic where friction varies")
    if antispin(preload)["margin"] < SAFETY:
        w.append(f"cam locking torque may SPIN the tool in its vees (anti-spin x"
                 f"{antispin(preload)['margin']}) — raise vee radius or preload path")
    if preload_from_servo(T_SERVO_CONT) < preload:
        w.append(f"servo can't reach the target preload at continuous torque "
                 f"({preload_from_servo(T_SERVO_CONT):.0f}N < {preload:.0f}N)")
    return w


if __name__ == "__main__":
    L = fidlock.PRINTER_TOOL
    sl, eff = self_locking(), efficiency()
    print(f"thread: dm{geo.POST_SHAFT_D:.0f} lead{geo.LEAD:.0f}mm  lead-angle {sl['lead_deg']}deg vs "
          f"friction {sl['friction_deg']}deg -> {'SELF-LOCKING' if sl['self_locking'] else 'BACK-DRIVES'} "
          f"(x{sl['margin']}); efficiency {eff*100:.0f}%")
    print(f"preload: {geo.ENGAGE_ANGLE:.0f}deg lock draws {geo.DRAW:.2f}mm; "
          f"target {TARGET_PRELOAD:.0f}N needs {servo_for_preload(TARGET_PRELOAD):.2f} N.m servo; "
          f"max at {T_SERVO_CONT} N.m cont = {preload_from_servo(T_SERVO_CONT):.0f}N "
          f"(stall {preload_from_servo(T_SERVO_STALL):.0f}N)")
    a = antispin(TARGET_PRELOAD)
    print(f"anti-spin: cinching reacts {a['reaction_Nm']} N.m on the tool; 120deg vees hold "
          f"{a['capacity_Nm']} N.m -> x{a['margin']} (geometry-set, independent of servo torque)")
    print()
    r = evaluate(TARGET_PRELOAD, L)
    m = r["margins"]
    print(f"vs service (moment {L.moment_Nm()*1000:.0f}Nmm, axial {L.axial_N():.1f}N) @ "
          f"{TARGET_PRELOAD:.0f}N preload:")
    print(f"  axial x{m['axial']:.0f}  liftoff x{m['moment_liftoff']:.1f}  "
          f"ultimate x{m['moment_ultimate']:.0f}  shear x{m['shear']:.0f}  "
          f"binds {r['binding']}  repeatability ~{r['repeatability_um']}um  "
          f"=> {'PASS' if r['pass'] else 'FAIL'}")
    fid = fidlock.evaluate(fidlock.Retention("kinematic + Fidlock", fidlock=True, rim_lock=False), L)
    print(f"  vs Fidlock coupling: liftoff x{fid['margins']['moment_liftoff']:.1f}, "
          f"repeatability ~{fid['repeatability_um']}um  (camlock trades a servo + anti-spin check "
          f"for commandable, higher preload)")
    for wv in validity(TARGET_PRELOAD, L):
        print(f"  ! {wv}")
    print("  [PREDICTION until a bench test anchors thread/contact friction + servo torque]")
