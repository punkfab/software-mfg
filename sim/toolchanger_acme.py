"""toolchanger_acme.py — statics for the SERVOLESS ACME tool changer: the wrist-roll (+ a dock
wrench) turns a self-locking, multi-start collar; the TPU vees are the preload spring; the wrist
CURRENT sets the clamp. Same min-of-modes discipline as sim/camlock_statics.py — numbers that aren't
yet measured (friction, TPU rate, servo stall) are flagged PREDICTION until a bench test anchors them.

The stack: arm piece (on the wrist horn, retains the ring) → TPU vee plate (snap-in, compliant) →
hex ACME locking ring (captive, free-spinning) → tool receiver (external ACME + 3 ball locators).
No changer servo — the wrist does the twist, the dock grounds the ring hex.

Five questions decide it:
  1. SELF-LOCK   — is the (multi-start!) lead shallow enough that lead angle < friction angle, so it
                   holds with the wrist UNPOWERED? Printed-plastic friction is high, so there's room.
  2. ENGAGEMENT  — an N-start thread lets the free-spinning ring catch from ANY orientation within
                   360/N° (the Gatorade-bottle trick) — no need to pre-index the ring. 4-start = 90°.
  3. PRELOAD     — the TPU vees ARE the spring: F = k_TPU · draw. Target 25 N (light tools).
  4. SET-BY-CURRENT — drive the wrist until Present_Load hits the threshold for 25 N; no hard stop.
  5. HOLD        — 25 N preload vs the tool's weight + a prying pull-off (does it stay on).

    python sim/toolchanger_acme.py            # evaluate + sweep -> build/toolchanger_acme.png
    python sim/toolchanger_acme.py --check    # gate: self-locks, reaches 25 N, engages <=90°, holds
"""
import math
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build"
G = 9.81
MM = 1e-3


@dataclass
class Params:
    mean_d_mm: float = 38.0        # ACME mean Ø (Gatorade-ish coarse thread)
    n_start: int = 4              # thread starts — engage within 360/n° of ANY ring orientation
    pitch_mm: float = 2.0         # crest-to-crest; lead = n_start * pitch
    mu_thread: float = 0.30       # printed plastic-on-plastic (PREDICTION — bench-measure)
    preload_N: float = 25.0       # target clamp (light tools; set by wrist current)
    tpu_k_Npmm: float = 25.0      # TPU vee axial spring rate (PREDICTION — coupon-measure)
    ball_pcd_mm: float = 30.0     # 3 ball locators at 120° on this circle
    tool_mass_kg: float = 0.30    # a gripper-ish tool
    wrist_stall_Nm: float = 1.0   # usable STS3215 wrist torque (conservative at low V; PREDICTION)
    safety: float = 1.5

    @property
    def lead_mm(self):
        return self.n_start * self.pitch_mm


# --- screw-thread mechanics (a power screw) -------------------------------------------------
def lead_angle(p: Params):
    return math.atan(p.lead_mm / (math.pi * p.mean_d_mm))          # rad


def friction_angle(p: Params):
    return math.atan(p.mu_thread)


def self_lock_margin_deg(p: Params):
    return math.degrees(friction_angle(p) - lead_angle(p))         # >0 = self-locking


def torque_for_preload(p: Params, F):
    """Wrist torque to make axial preload F through the ACME ratio (lock direction, λ+φ)."""
    rm = 0.5 * p.mean_d_mm * MM
    return F * rm * math.tan(lead_angle(p) + friction_angle(p))    # N·m


def unlock_torque(p: Params, F):
    """Torque to DRIVE it open against self-lock (|λ-φ|). Positive here means it can't back-drive on
    its own — you must actively turn it — which is the whole point."""
    rm = 0.5 * p.mean_d_mm * MM
    return F * rm * abs(math.tan(lead_angle(p) - friction_angle(p)))


def present_load_for(p: Params, T):
    """Approx STS3215 Present_Load (0..1000) for a shaft torque T — the current threshold the motion
    program watches to STOP at preload. Rough (load ∝ current ∝ torque); calibrate on the bench."""
    return max(0.0, min(1000.0, 1000.0 * T / p.wrist_stall_Nm))


# --- preload from the TPU spring (geometry, not torque) -------------------------------------
def draw_for_preload(p: Params, F):
    return F / p.tpu_k_Npmm                                        # mm the ring must pull the tool in


def lock_rotation_deg(p: Params, F):
    return 360.0 * draw_for_preload(p, F) / p.lead_mm             # wrist twist to reach preload


def engagement_deg(p: Params):
    return 360.0 / p.n_start                                       # worst-case twist to CATCH a start


# --- hold (does 25 N keep the tool on) ------------------------------------------------------
def axial_pulloff_N(p: Params):
    return p.preload_N                                             # self-locked screw holds ~its preload


def liftoff_moment_Nm(p: Params):
    """Prying: a moment lifts one side of the 3-ball seat until preload is overcome at the far vee."""
    return p.preload_N * (p.ball_pcd_mm * MM)


def evaluate(p: Params):
    T = torque_for_preload(p, p.preload_N)
    weight = p.tool_mass_kg * G
    m = {
        "self_lock_deg": self_lock_margin_deg(p),
        "lead_deg": math.degrees(lead_angle(p)),
        "friction_deg": math.degrees(friction_angle(p)),
        "engage_deg": engagement_deg(p),
        "draw_mm": draw_for_preload(p, p.preload_N),
        "lock_rotation_deg": lock_rotation_deg(p, p.preload_N),
        "wrist_torque_Nm": T,
        "present_load": present_load_for(p, T),
        "unlock_torque_Nm": unlock_torque(p, p.preload_N),
        "axial_pulloff_N": axial_pulloff_N(p),
        "axial_margin": axial_pulloff_N(p) / (weight * p.safety),
        "liftoff_Nm": liftoff_moment_Nm(p),
    }
    m["pass"] = bool(m["self_lock_deg"] > 3.0                     # self-locks with a real margin
                     and T < p.wrist_stall_Nm                     # wrist can reach 25 N
                     and m["engage_deg"] <= 90.0 + 1e-6           # 4-start -> no ring pre-index
                     and m["axial_margin"] >= 1.0)                # holds its own weight × safety
    return m


def _plot(p: Params, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5))

    # self-lock margin + lock rotation vs LEAD (the one real knob): shows the design window
    leads = [x / 10 for x in range(5, 161)]                       # 0.5..16 mm lead
    marg = [self_lock_margin_deg(Params(**{**p.__dict__, "n_start": 1, "pitch_mm": L})) for L in leads]
    rot = [lock_rotation_deg(Params(**{**p.__dict__, "n_start": 1, "pitch_mm": L}), p.preload_N)
           for L in leads]
    a1.plot(leads, marg, color="#3fae5a", label="self-lock margin (°)")
    a1.axhline(0, color="#e04b4b", ls="--", lw=1, label="back-drive boundary")
    a1.set_xlabel("thread lead (mm/turn)"); a1.set_ylabel("self-lock margin (°)", color="#3fae5a")
    a1r = a1.twinx(); a1r.plot(leads, rot, color="#4a90d9"); a1r.set_ylabel("lock twist (°)", color="#4a90d9")
    a1r.set_ylim(0, 720)
    op = p.lead_mm
    a1.axvline(op, color="k", ls=":", lw=1); a1.annotate(f"op: {op:.0f}mm lead\n({p.n_start}-start)",
              (op, 2), (op + 1, 4), fontsize=8)
    a1.set_title("self-lock vs lead (printed plastic has room)"); a1.legend(loc="upper right", fontsize=8)

    # engagement angle vs number of starts (Gatorade trick)
    ns = [1, 2, 3, 4, 6]
    a2.bar([str(n) for n in ns], [360.0 / n for n in ns], color="#8ab4d8")
    a2.axhline(90, color="#e08a3c", ls="--", label="no-pre-index target (≤90°)")
    a2.set_xlabel("thread starts"); a2.set_ylabel("worst-case twist to catch (°)")
    a2.set_title("multi-start = engage from any ring orientation"); a2.legend(fontsize=8)

    m = evaluate(p)
    fig.suptitle(f"servoless ACME changer:  {p.n_start}-start, {p.lead_mm:.0f}mm lead, Ø{p.mean_d_mm:.0f}  "
                 f"→  self-lock {m['self_lock_deg']:.0f}° margin, {p.preload_N:.0f}N @ "
                 f"{m['wrist_torque_Nm']*1000:.0f}mN·m (Present_Load≈{m['present_load']:.0f}), "
                 f"lock in {m['lock_rotation_deg']:.0f}°  [{'PASS' if m['pass'] else 'FAIL'}]", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    BUILD.mkdir(exist_ok=True)
    fig.savefig(path, dpi=110)
    return path


def main():
    p = Params()
    m = evaluate(p)
    print(f"servoless ACME tool changer — {p.n_start}-start, lead {p.lead_mm:.0f}mm, Ø{p.mean_d_mm:.0f}mm, "
          f"μ={p.mu_thread} (PREDICTION):")
    print(f"  1. SELF-LOCK : lead {m['lead_deg']:.1f}° < friction {m['friction_deg']:.1f}° "
          f"-> margin {m['self_lock_deg']:.1f}°  ({'holds unpowered' if m['self_lock_deg']>0 else 'BACK-DRIVES'})")
    print(f"  2. ENGAGE    : {m['engage_deg']:.0f}° worst-case to catch a start (ring needs NO pre-index)")
    print(f"  3. PRELOAD   : {p.preload_N:.0f}N from TPU k={p.tpu_k_Npmm}N/mm -> draw {m['draw_mm']:.2f}mm "
          f"= {m['lock_rotation_deg']:.0f}° of wrist twist")
    print(f"  4. SET-BY-I  : wrist torque {m['wrist_torque_Nm']*1000:.0f}mN·m -> stop at "
          f"Present_Load ≈ {m['present_load']:.0f}/1000 (bench-calibrate the exact value)")
    print(f"  5. HOLD      : axial pull-off {m['axial_pulloff_N']:.0f}N vs tool weight "
          f"{p.tool_mass_kg*G:.1f}N×{p.safety} -> margin ×{m['axial_margin']:.1f}; "
          f"prying liftoff {m['liftoff_Nm']*1000:.0f}mN·m")
    print(f"  unlock needs {m['unlock_torque_Nm']*1000:.0f}mN·m of active drive (can't self-open)")
    print(f"  => {'PASS' if m['pass'] else 'FAIL'} (numbers are PREDICTION until a coupon/bench test)")
    _plot(p, str(BUILD / "toolchanger_acme.png"))
    print(f"  wrote {BUILD/'toolchanger_acme.png'}")
    return 0


def check():
    problems = []
    p = Params()
    m = evaluate(p)
    if m["self_lock_deg"] <= 3.0:
        problems.append(f"not self-locking with margin (got {m['self_lock_deg']:.1f}°)")
    if m["wrist_torque_Nm"] >= p.wrist_stall_Nm:
        problems.append(f"wrist can't reach 25N ({m['wrist_torque_Nm']:.2f} >= {p.wrist_stall_Nm})")
    if m["engage_deg"] > 90 + 1e-6:
        problems.append(f"4-start should engage <=90° (got {m['engage_deg']:.0f})")
    if m["axial_margin"] < 1.0:
        problems.append(f"hold margin < 1 ({m['axial_margin']:.2f})")
    # a steep single-start (lead 16mm) must correctly read as marginal/back-driving vs the 4-start op
    steep = evaluate(Params(n_start=1, pitch_mm=16.0))
    if steep["self_lock_deg"] >= m["self_lock_deg"]:
        problems.append("steep lead should have LESS self-lock margin than the operating point")
    if problems:
        for x in problems:
            print("FAIL:", x)
        return 1
    print(f"PASS: servoless ACME changer self-locks ({m['self_lock_deg']:.0f}° margin), reaches 25N at "
          f"{m['wrist_torque_Nm']*1000:.0f}mN·m (Present_Load≈{m['present_load']:.0f}), engages ≤90°, "
          f"holds ×{m['axial_margin']:.1f} — hardware-free")
    return 0


if __name__ == "__main__":
    sys.exit(check() if "--check" in sys.argv else main())
