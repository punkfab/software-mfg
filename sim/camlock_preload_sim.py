"""camlock_preload_sim.py — does the servo draw-lock HOLD its clamp force, or does it need a
compliant element (a Belleville / wave spring) in the draw path? MuJoCo, geometry + preload
target imported from parts/_camlock.py + sim/camlock_statics.py.

The physics the sim makes concrete: a self-locking thread + a position-hold servo maintain
POSITION, not FORCE. Preload is stored as F0 = k * interference. A rigid stack has huge k,
so the interference holding 60 N is only ~20 um — and any dimensional loss (PLA creep,
thermal, wear, settling; realistically 10s-100s of um) eats straight into it and the clamp
collapses. A soft spring in series stores the same 60 N over ~2 mm, so the same loss barely
dents it. A continuously-powered TORQUE-hold servo also holds force, but burns power and has
to back-drive a self-locking thread to re-tighten (stiction) — and holds nothing once off.

Lumped 1-DOF model (the essential axial physics):
    servo-set nut --[draw spring k]--> tool --[kinematic-seat hard stop]--> arm
The seat is a joint lower-limit at z=0; the clamp force is that limit's constraint force.
Creep/thermal/wear = the interference shrinks. Pull-off (prying moment + weight + process) =
an external +z force. Gap = the tool lifts off the seat (clamp -> 0, registration lost).

  python sim/camlock_preload_sim.py            # interactive: pull on the tool, watch it gap
  python sim/camlock_preload_sim.py --demo     # creep-retention + vibration plots -> build/
  python sim/camlock_preload_sim.py --selftest # assert the physics, no window (CI)
"""
import os
import sys
from dataclasses import dataclass
from pathlib import Path

_MODE = "demo" if "--demo" in sys.argv else "selftest" if "--selftest" in sys.argv else "interactive"
os.environ.setdefault("MUJOCO_GL", "glfw" if _MODE == "interactive" else "osmesa")

import numpy as np  # noqa: E402
import mujoco  # noqa: E402

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "parts"))
sys.path.insert(0, str(_ROOT / "sim"))
import _camlock as geo  # noqa: E402
import camlock_statics as cs  # noqa: E402
import coupling_statics as cps  # noqa: E402  # for CONTACT_K (the plastic seat stiffness)


@dataclass
class Params:
    dt: float = 1e-5                         # small: the rigid stack is stiff; keeps the seat hard
    F0: float = cs.TARGET_PRELOAD            # commanded preload (N), from the statics
    k_rigid: float = cps.CONTACT_K * 1e6     # plastic seat/thread stack stiffness (N/m); 3 N/um
    k_spring: float = 3.0e4                  # Belleville / wave spring in the draw path (N/m)
    m_tool: float = 0.15                     # tool + payload hung on the coupling (kg)
    zeta: float = 0.9                        # draw-spring damping ratio (clean settle, stable)
    torque_damp: float = 30.0                # damping when there is no spring (torque-hold)
    seat_solref: float = 5e-5                # seat stop time-const: hard vs the rigid stack
    settle_s: float = 0.08

    def interference(self, k):
        return self.F0 / k                   # elastic squeeze that stores F0 at stiffness k

    def damping(self, k):
        # scale damping to each config's stiffness so a stiff spring stays numerically stable
        return self.zeta * 2 * (max(k, 1.0) * self.m_tool) ** 0.5 if k > 0 else self.torque_damp


def make_model(p: Params, k, interference):
    """Slide joint (tool z) pulled onto a z=0 seat stop by a draw spring of stiffness k with
    `interference` of squeeze. k<=0 -> no spring (torque-hold mode drives force externally)."""
    stiff = max(k, 0.0)
    springref = -interference if k > 0 else 0.0     # rest below the seat -> pulls into it
    xml = f"""
    <mujoco>
      <option timestep="{p.dt}" integrator="implicitfast" gravity="0 0 0"/>
      <worldbody>
        <geom name="arm" type="box" size="0.03 0.03 0.005" pos="0 0 -0.005" rgba="0.3 0.3 0.34 1"/>
        <body name="tool" pos="0 0 0">
          <joint name="z" type="slide" axis="0 0 1" range="0 0.02" limited="true"
                 stiffness="{stiff}" springref="{springref}" damping="{p.damping(k)}"
                 solreflimit="{p.seat_solref} 1"/>
          <geom type="box" size="0.025 0.025 0.01" pos="0 0 0.01" mass="{p.m_tool}"
                rgba="0.15 0.55 0.75 1"/>
        </body>
      </worldbody>
    </mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def _dof(model):
    return model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "z")]


def clamp_force(model, data, dof):
    """The seat's constraint (limit) reaction = the live clamp force. Zero once the tool gaps."""
    return abs(float(data.qfrc_constraint[dof]))


def settle_clamp(p: Params, config, creep_um=0.0, f_ext=0.0):
    """Settle one config with a given creep loss + steady external pull-off; return clamp (N)
    and whether the seat is still closed."""
    k, torque_hold = _config_k(p, config)
    interf = max(0.0, p.interference(k) - creep_um * 1e-6) if k > 0 else 0.0
    model = make_model(p, k, interf)
    data = mujoco.MjData(model)
    dof = _dof(model)
    for _ in range(int(p.settle_s / p.dt)):
        data.qfrc_applied[dof] = f_ext + (-p.F0 if torque_hold else 0.0)
        mujoco.mj_step(model, data)
    seated = data.qpos[dof] < 1e-5
    return clamp_force(model, data, dof), seated


def _config_k(p: Params, config):
    if config == "rigid":
        return p.k_rigid, False
    if config == "compliant":
        return p.k_spring, False
    if config == "torque_hold":
        return -1.0, True          # no spring; a constant servo force holds the draw
    raise ValueError(config)


def creep_sweep(p: Params, creeps_um):
    """Preload retained vs a dimensional loss, for each config."""
    return {cfg: [settle_clamp(p, cfg, creep_um=c)[0] for c in creeps_um]
            for cfg in ("rigid", "compliant", "torque_hold")}


def vibration_run(p: Params, config, creep_um, amp_N, freq_hz, secs=0.6):
    """After a realistic creep, shake the tool with an oscillating pull-off; log clamp + gap."""
    k, torque_hold = _config_k(p, config)
    interf = max(0.0, p.interference(k) - creep_um * 1e-6) if k > 0 else 0.0
    model = make_model(p, k, interf)
    data = mujoco.MjData(model)
    dof = _dof(model)
    for _ in range(int(p.settle_s / p.dt)):        # settle to the (crept) preload first
        data.qfrc_applied[dof] = -p.F0 if torque_hold else 0.0
        mujoco.mj_step(model, data)
    log = {k2: [] for k2 in ("t", "clamp", "fext", "gap")}
    n = int(secs / p.dt)
    for i in range(n):
        t = i * p.dt
        fext = amp_N * max(0.0, np.sin(2 * np.pi * freq_hz * t))   # pull-off only (one-sided)
        data.qfrc_applied[dof] = fext + (-p.F0 if torque_hold else 0.0)
        mujoco.mj_step(model, data)
        log["t"].append(t); log["fext"].append(fext)
        log["clamp"].append(clamp_force(model, data, dof))
        log["gap"].append(float(data.qpos[dof]) > 1e-5)
    return log


# --- modes -----------------------------------------------------------------------------------
def selftest():
    p = Params()
    dof_um = 40.0                       # a realistic creep+thermal loss over a work session
    base = {c: settle_clamp(p, c, 0.0)[0] for c in ("rigid", "compliant", "torque_hold")}
    crept = {c: settle_clamp(p, c, dof_um)[0] for c in ("rigid", "compliant", "torque_hold")}
    interf_rigid_um = p.interference(p.k_rigid) * 1e6
    # physics: all three seat to ~F0 with no creep
    ok_base = all(abs(base[c] - p.F0) < 0.1 * p.F0 for c in base)
    # rigid preload lives in a ~F0/k_rigid squeeze; a 40um loss > that -> preload collapses
    ok_rigid = crept["rigid"] < 0.25 * p.F0
    # the spring retains almost all preload (loss ~ k_spring*creep, tiny)
    ok_comp = crept["compliant"] > 0.9 * p.F0
    # torque-hold holds force (constant source), independent of the dimensional loss
    ok_torque = abs(crept["torque_hold"] - p.F0) < 0.1 * p.F0
    ok = ok_base and ok_rigid and ok_comp and ok_torque
    print(f"preload maintenance (F0={p.F0:.0f}N, rigid squeeze only {interf_rigid_um:.0f}um):")
    print(f"  no creep : rigid {base['rigid']:.0f}N  compliant {base['compliant']:.0f}N  "
          f"torque-hold {base['torque_hold']:.0f}N")
    print(f"  +{dof_um:.0f}um loss: rigid {crept['rigid']:.0f}N  compliant {crept['compliant']:.0f}N  "
          f"torque-hold {crept['torque_hold']:.0f}N")
    print(f"  => rigid position-hold LOSES the clamp on creep; a {p.k_spring:.0e} N/m spring "
          f"holds it; torque-hold holds it but needs power")
    print("PASS: compliance (or continuous torque-hold) is required to maintain clamp force"
          if ok else "FAIL: preload-retention physics not reproduced")
    return 0 if ok else 1


def demo():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    p = Params()
    creeps = np.linspace(0, 100, 21)
    sweep = creep_sweep(p, creeps)
    interf_rigid_um = p.interference(p.k_rigid) * 1e6

    # a fair pull-off amplitude ~ the preload; shake after a 40um creep
    vib = {c: vibration_run(p, c, 40.0, amp_N=0.9 * p.F0, freq_hz=25.0)
           for c in ("rigid", "compliant")}

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
    for c, style in (("rigid", "-o"), ("compliant", "-s"), ("torque_hold", "-^")):
        ax1.plot(creeps, sweep[c], style, ms=3, label=c.replace("_", "-"))
    ax1.axhline(0.5 * p.F0, ls=":", c="0.6"); ax1.set_xlabel("dimensional loss: creep/thermal/wear (um)")
    ax1.set_ylabel("clamp force retained (N)")
    ax1.set_title(f"preload retention (rigid squeeze = {interf_rigid_um:.0f}um)")
    ax1.legend(); ax1.grid(alpha=0.3)

    for c, col in (("rigid", "#c23b3b"), ("compliant", "#2b7fb4")):
        ax2.plot(vib[c]["t"], vib[c]["clamp"], color=col, label=f"{c} clamp")
    ax2.plot(vib["rigid"]["t"], vib["rigid"]["fext"], color="0.5", ls="--", lw=0.8, label="pull-off")
    ax2.set_xlabel("time (s)"); ax2.set_ylabel("force (N)")
    ax2.set_title("vibration after 40um creep (does the seat gap?)")
    ax2.legend(fontsize=8); ax2.grid(alpha=0.3)
    fig.tight_layout()
    out = _ROOT / "build" / "camlock_preload.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110)
    rg = 100 * (1 - sweep["rigid"][8] / p.F0)     # ~40um point
    cg = 100 * (1 - sweep["compliant"][8] / p.F0)
    print(f"at 40um loss: rigid keeps {sweep['rigid'][8]:.0f}N ({rg:.0f}% lost), "
          f"compliant keeps {sweep['compliant'][8]:.0f}N ({cg:.0f}% lost) -> {out}")


def interactive():
    from mujoco import viewer
    p = Params()
    model = make_model(p, p.k_spring, p.interference(p.k_spring))
    data = mujoco.MjData(model)
    print("compliant draw-lock: drag the tool up to feel it unseat (clamp -> 0 when it gaps)")
    viewer.launch(model, data)


if __name__ == "__main__":
    sys.exit({"selftest": selftest, "demo": demo, "interactive": interactive}[_MODE]() or 0)
