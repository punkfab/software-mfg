"""assembly_insert.py — capture-envelope + acceptance gate for the snap-fit peg-in-socket assembly.

This is the Manufacturing-for-Design payoff of the two CAD parts (parts/snap_peg.py, snap_socket.py):
it answers "will a cheap, imprecise arm actually seat this part, and how much does the DESIGNED lead-in
buy us?" — not "did the robot do it once."

MODEL (v1, geometric/analytic — fast + deterministic; a MuJoCo physical-insertion pass is the v2 upgrade):
  The arm presents the peg tip with a pose error: lateral offset (2-D isotropic Gaussian, per-axis sigma
  sigma_d) and tilt (Gaussian, sigma sigma_th). The socket mouth chamfer funnels the tip home for any
  lateral offset up to the CAPTURE RADIUS (from the CAD: chamfer depth + half the slip clearance), and
  tolerates tilt up to THETA_MAX (the tapered nose still enters the mouth). Success probability is the
  mass of the pose-error distribution inside that capture window — lateral via the Rayleigh CDF,
  angular via the normal CDF.

ACCEPTANCE GATE (verified-by-construction): capture_radius >= MARGIN * sigma_d. If the designed chamfer
clears the base's pose scatter by the margin, the assembly is robust BY DESIGN; if not, the fix is a
bigger chamfer or a tighter base — and the sweep shows exactly how much of each.

All mating geometry is imported from parts/_snapfit (single source of truth — never re-typed here).

    python3 sim/assembly_insert.py            # sweep + plots -> build/assembly_envelope.png
    python3 sim/assembly_insert.py --check    # acceptance gate (capture R >= margin*sigma_d)
    python3 sim/assembly_insert.py --selftest # build + evaluate, asserts finite, no plot
"""
import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "parts"))
import _snapfit as S


@dataclass
class Params:
    # --- reality inputs (ASSUMED until measured on hardware, then written back) ---------------
    sigma_d_mm: float = 2.0      # base+arm LATERAL pose sigma, per axis. ASSUMPTION for a cheap omni+arm.
    sigma_th_deg: float = 1.5    # base+arm angular (tilt) pose sigma.
    margin: float = 1.5          # how many sigmas the capture window must clear (the gate's safety factor)
    # --- design knob (the affordance) --------------------------------------------------------
    chamfer_depth_mm: float = S.CHAMFER_DEPTH   # swept; default = the current CAD value


def capture_radius(p: Params) -> float:
    """Lateral catch radius the mouth provides (imported reach + half the slip clearance)."""
    return S.capture_radius(p.chamfer_depth_mm)


def theta_max_deg(p: Params) -> float:
    """Max peg tilt the nose+mouth still admits: the tapered nose (NOSE_R over NOSE_LEN) can enter a
    mouth that is (r_mouth - NOSE_R) wider than the tip. Geometric bound, calibratable on the bench."""
    r_mouth = S.BORE_D / 2 + p.chamfer_depth_mm
    return math.degrees(math.atan2(r_mouth - S.NOSE_R, S.NOSE_LEN))


def p_success(p: Params) -> float:
    """P(pose error lands inside the capture window). Lateral: 2-D isotropic Gaussian -> Rayleigh CDF
    P(r<=R) = 1 - exp(-R^2 / 2 sigma^2). Angular: two-sided normal, P(|t|<=T) = erf(T / (sqrt2 sigma))."""
    R = capture_radius(p)
    p_lat = 1.0 - math.exp(-(R * R) / (2.0 * p.sigma_d_mm ** 2)) if p.sigma_d_mm > 0 else 1.0
    T = theta_max_deg(p)
    p_ang = math.erf(T / (math.sqrt(2.0) * p.sigma_th_deg)) if p.sigma_th_deg > 0 else 1.0
    return p_lat * p_ang


def gate(p: Params):
    """Verified-by-construction acceptance: capture radius clears margin * sigma_d."""
    R, need = capture_radius(p), p.margin * p.sigma_d_mm
    return R >= need, R, need


def _fmt(p: Params) -> str:
    ok, R, need = gate(p)
    return (f"chamfer={p.chamfer_depth_mm:.1f}mm  capture_R={R:.2f}mm  theta_max={theta_max_deg(p):.1f}deg"
            f"  P(seat)={p_success(p)*100:.1f}%  gate(R>= {need:.1f})={'PASS' if ok else 'FAIL'}"
            f"  [sigma_d={p.sigma_d_mm}mm, margin={p.margin}]")


def sweep_chamfer(p: Params, depths):
    return [(d, capture_radius(Params(**{**p.__dict__, "chamfer_depth_mm": d})),
             p_success(Params(**{**p.__dict__, "chamfer_depth_mm": d}))) for d in depths]


def make_plots(p: Params):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    depths = [i * 0.25 for i in range(0, 25)]           # 0 .. 6 mm chamfer
    data = sweep_chamfer(p, depths)
    dd = [d for d, _, _ in data]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.4))

    # (a) the affordance lever: P(seat) vs chamfer depth, current design + the "no chamfer" baseline
    ax1.plot(dd, [s * 100 for _, _, s in data], color="#1b6ca8", lw=2)
    cur = p_success(p) * 100
    ax1.scatter([p.chamfer_depth_mm], [cur], color="#c0392b", zorder=5,
                label=f"current design: {cur:.0f}%")
    base = p_success(Params(**{**p.__dict__, "chamfer_depth_mm": 0.0})) * 100
    ax1.scatter([0.0], [base], color="#7f8c8d", zorder=5, label=f"no lead-in: {base:.0f}%")
    ax1.set_xlabel("mouth chamfer depth (mm)  — the designed affordance")
    ax1.set_ylabel("P(seat) at base sigma_d=%.1fmm  (%%)" % p.sigma_d_mm)
    ax1.set_title("the lead-in is the product"); ax1.set_ylim(0, 101); ax1.legend(fontsize=8); ax1.grid(alpha=.3)

    # (b) the acceptance gate: capture radius vs the margin*sigma requirement line
    ax2.plot(dd, [R for _, R, _ in data], color="#1b6ca8", lw=2, label="capture radius (design)")
    ax2.axhline(p.margin * p.sigma_d_mm, color="#c0392b", ls="--",
                label=f"gate: margin*sigma_d = {p.margin*p.sigma_d_mm:.1f}mm")
    ax2.axvline(p.chamfer_depth_mm, color="#7f8c8d", ls=":", lw=1)
    ax2.set_xlabel("mouth chamfer depth (mm)"); ax2.set_ylabel("lateral capture radius (mm)")
    ax2.set_title("verified-by-construction gate"); ax2.legend(fontsize=8); ax2.grid(alpha=.3)

    fig.suptitle("snap-fit assembly capture envelope  —  " + _fmt(p), fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    os.makedirs("build", exist_ok=True)
    fig.savefig("build/assembly_envelope.png", dpi=110)
    print("wrote build/assembly_envelope.png")


def main():
    p = Params()
    print(_fmt(p))
    make_plots(p)


def check():
    p = Params()
    ok, R, need = gate(p)
    print(_fmt(p))
    if ok:
        print(f"PASS: designed capture radius {R:.2f}mm clears margin*sigma_d {need:.1f}mm "
              f"-> the snap-fit seats by design at the assumed base scatter (P={p_success(p)*100:.0f}%).")
    else:
        short = need - R
        print(f"FAIL: capture radius {R:.2f}mm is {short:.2f}mm short of margin*sigma_d {need:.1f}mm. "
              f"Fix: deepen the chamfer to >= {need - (S.BORE_D-S.PEG_D)/2:.1f}mm, or tighten the base.")
    sys.exit(0 if ok else 1)


def selftest():
    for depth in (0.0, S.CHAMFER_DEPTH, 6.0):
        p = Params(chamfer_depth_mm=depth)
        v = p_success(p)
        assert 0.0 <= v <= 1.0 and math.isfinite(v), v
        assert capture_radius(p) >= 0 and math.isfinite(theta_max_deg(p))
    # monotonic: a deeper lead-in never lowers success
    seq = [p_success(Params(chamfer_depth_mm=d)) for d in (0, 1, 2, 3, 4, 5, 6)]
    assert all(b >= a - 1e-9 for a, b in zip(seq, seq[1:])), seq
    print("selftest OK:", _fmt(Params()))


if __name__ == "__main__":
    ({"--check": check, "--selftest": selftest}.get(sys.argv[1] if len(sys.argv) > 1 else "", main))()
