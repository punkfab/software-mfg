"""mobile_base_mj.py — the mobile base as a MuJoCo rigid body, so tip-over is a SIMULATED
outcome, not an assumption.

`mobile_base.py` predicts the tip limit from a moment balance. This builds the base as a
free body resting on its wheel/foot contacts, hangs the arm+payload on a slide, and lets
the physics engine decide when it tips as the arm reaches out. The payoff is a **cross-check**:
does MuJoCo tip the base at the same reach the analytic model predicts? Two independent
models agreeing is the round-trip discipline (design ↔ sim) applied to the base statics.

The arm is a lumped mass on a horizontal slide — its internal kinematics don't change the
base tip-over, only its CG does. Contacts are the support points only (the deck is visual),
so the base pivots about the real support polygon edge. Reach direction = +x.
"""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
sys.path.insert(0, str(_ROOT / "sim"))
import mobile_base as mb  # noqa: E402

_G = mujoco.mjtGeom
ONSET_DEG = 3.0           # base tilt first exceeding this = tip ONSET (CG crossed the edge)
TIP_DEG = 15.0            # tilt beyond this = fully tipped over


def _support_points(d: mb.BaseDesign):
    """Ground-contact points (x, y) in the base frame — the support polygon vertices."""
    if d.layout == "diff2":
        return [(d.x_rear, +d.half_track), (d.x_rear, -d.half_track), (d.x_front, 0.0)]
    return [(sx, sy) for sx in (d.x_front, d.x_rear) for sy in (+d.half_track, -d.half_track)]


def _lumped_arm(load: mb.ArmLoad):
    """Arm+payload as one mass at an effective (x0, height) — CG is all that matters here."""
    m = load.mass_arm_kg + load.payload_kg
    x0 = (load.mass_arm_kg * (load.arm_mount_x + load.arm_cg_dx)
          + load.payload_kg * load.ee_reach_x) / m
    h = (load.mass_arm_kg * (load.arm_mount_height + load.arm_cg_dz)
         + load.payload_kg * load.ee_height) / m
    return m, x0, h


def predict_tip_reach(d: mb.BaseDesign, load: mb.ArmLoad):
    """Analytic: extra forward slide `s` (m) of the lumped arm mass at which the combined CG
    crosses the front support edge (x_front) — where the base should begin to tip."""
    mA, x0, _ = _lumped_arm(load)
    mtot = d.mass_base_kg + mA
    # x_cg(s) = (m_base*0 + mA*(x0 + s)) / mtot == x_front  ->  s
    return d.x_front * mtot / mA - x0


def build_spec(d: mb.BaseDesign, load: mb.ArmLoad, reach_range=0.6):
    spec = mujoco.MjSpec()
    spec.option.timestep = 0.002
    spec.compiler.autolimits = True
    wb = spec.worldbody
    wb.add_light(pos=[0.2, -0.2, 0.8], dir=[-0.2, 0.2, -1])
    floor = wb.add_geom()
    floor.name, floor.type, floor.size = "floor", _G.mjGEOM_PLANE, [0, 0, 0.05]
    floor.rgba, floor.friction = [0.3, 0.32, 0.36, 1], [1.0, 0.02, 0.001]

    r = d.wheel_radius_m
    base = wb.add_body()
    base.name, base.pos = "base", [0, 0, r]
    base.add_freejoint()

    # base mass at its CG height (world cg_height -> local cg_height - r)
    bm = base.add_geom()
    bm.name, bm.type, bm.size, bm.pos = "base_mass", _G.mjGEOM_BOX, [0.02, 0.02, 0.02], [0, 0, d.cg_height_m - r]
    bm.mass, bm.contype, bm.conaffinity, bm.rgba = d.mass_base_kg, 0, 0, [0.35, 0.4, 0.46, 1]
    # visual deck
    deck = base.add_geom()
    deck.name, deck.type = "deck", _G.mjGEOM_BOX
    deck.size, deck.pos = [(d.x_front - d.x_rear) / 2, d.half_track, 0.003], [(d.x_front + d.x_rear) / 2, 0, 0]
    deck.mass, deck.contype, deck.conaffinity, deck.rgba = 0.001, 0, 0, [0.5, 0.53, 0.58, 1]
    # support contacts (wheels / feet) — the only things that touch the floor
    for i, (x, y) in enumerate(_support_points(d)):
        w = base.add_geom()
        w.name, w.type, w.size, w.pos = f"support{i}", _G.mjGEOM_SPHERE, [r, 0, 0], [x, y, 0]
        w.mass, w.friction, w.rgba = 0.05, [1.0, 0.02, 0.001], [0.15, 0.15, 0.17, 1]

    # arm+payload = a lumped mass on a horizontal slide (reach)
    mA, x0, hA = _lumped_arm(load)
    arm = base.add_body()
    arm.name, arm.pos = "arm", [x0, 0, 0]
    aj = arm.add_joint()
    aj.name, aj.type, aj.axis, aj.range = "reach_x", mujoco.mjtJoint.mjJNT_SLIDE, [1, 0, 0], [0.0, reach_range]
    aj.armature = 0.5
    ag = arm.add_geom()
    ag.name, ag.type, ag.size, ag.pos = "arm_mass", _G.mjGEOM_BOX, [0.02, 0.02, 0.02], [0, 0, hA - r]
    ag.mass, ag.contype, ag.conaffinity, ag.rgba = mA, 0, 0, [0.9, 0.6, 0.15, 1]

    act = spec.add_actuator()
    act.name, act.target, act.trntype = "reach", "reach_x", mujoco.mjtTrn.mjTRN_JOINT
    act.set_to_position(kp=2000, kv=80)
    act.forcerange = [-200, 200]
    return spec


def _tilt_deg(model, data, base_id):
    """Angle of the base's local z-axis from world vertical (degrees)."""
    zaxis = data.xmat[base_id].reshape(3, 3)[:, 2]
    return float(np.degrees(np.arccos(np.clip(zaxis @ np.array([0, 0, 1.0]), -1, 1))))


def run_reach(d: mb.BaseDesign, load: mb.ArmLoad, ramp_s=3.0, reach_range=0.6):
    """Settle the base, then ramp the arm reach 0 -> reach_range. Report the reach at which
    it tips (or 'stable'), the peak tilt, and the analytic prediction for comparison."""
    model = build_spec(d, load, reach_range).compile()
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    for _ in range(200):                      # settle onto the contacts
        mujoco.mj_step(model, data)
    settle_tilt = _tilt_deg(model, data, bid)

    steps = int(ramp_s / model.opt.timestep)
    onset_reach = full_reach = None
    max_tilt = settle_tilt
    for k in range(steps):
        data.ctrl[0] = reach_range * (k / steps)      # commanded reach
        mujoco.mj_step(model, data)
        tilt = _tilt_deg(model, data, bid)
        max_tilt = max(max_tilt, tilt)
        if tilt > ONSET_DEG and onset_reach is None:
            onset_reach = float(data.qpos[-1])        # reach slide value at tip ONSET
        if tilt > TIP_DEG and full_reach is None:
            full_reach = float(data.qpos[-1])
            break
    return {"design": d.name, "settle_tilt_deg": round(settle_tilt, 2),
            "tips": onset_reach is not None,
            "tip_reach_m": None if onset_reach is None else round(onset_reach, 3),
            "predict_tip_reach_m": round(predict_tip_reach(d, load), 3),
            "went_over": full_reach is not None, "max_tilt_deg": round(max_tilt, 1)}


def reach_stable(d: mb.BaseDesign, load: mb.ArmLoad, reach_m):
    """Quasi-static: hold the arm at a specific reach and report the settled base tilt —
    the operating check (is the base steady at the real working reach?)."""
    model = build_spec(d, load, reach_range=max(0.05, reach_m)).compile()
    data = mujoco.MjData(model)
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    for _ in range(200):
        mujoco.mj_step(model, data)
    data.ctrl[0] = reach_m
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    return {"reach_m": reach_m, "tilt_deg": round(_tilt_deg(model, data, bid), 2)}


if __name__ == "__main__":
    print("MuJoCo dynamic tip-over — arm reaches out until the base tips (vs analytic prediction)\n")
    for key, d in mb.DESIGNS.items():
        r = run_reach(d, mb.PRINTER_PICK)
        tip = f"tips @ {r['tip_reach_m']}m" if r["tips"] else "stable to full reach"
        print(f"  {d.name:28s} settle {r['settle_tilt_deg']:.1f}° | {tip:22s} "
              f"(predict {r['predict_tip_reach_m']}m) | peak {r['max_tilt_deg']:.0f}°")
