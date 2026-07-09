#!/usr/bin/env python3
"""Gate: the servo-driven central draw-lock changer — 3 parts print clean, the interrupted
thread actually inserts + grabs, and the statics say it self-locks, doesn't spin its own
tool, and secures it at the worst prying pose.

Ties parts/_camlock.py (geometry) to sim/camlock_statics.py (physics), the same way
coupling_check ties the Fidlock coupling together:

  - all 3 faces (tool, cam ring, arm) are single watertight solids;
  - the locator is EQUAL 120deg (symmetric — this design keys via the pogo, not the seat);
  - thread engages: cam catches reach INSIDE the lug OD (grab) while the gaps clear it
    (insert), and each lug is narrower than a gap so it passes;
  - known-physics: the lead is self-locking (lead angle < friction angle); the anti-spin
    margin is GEOMETRY-set (same at any preload — capacity and reaction both scale with it);
    preload is linear in servo torque; the design holds the service moment and beats the
    Fidlock's registration margin (its whole reason to add a servo).
"""

import subprocess
import sys
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "parts"))
import _camlock as geo  # noqa: E402
import camlock_statics as cs  # noqa: E402
import coupling_statics as fid  # noqa: E402

EXPORTS = ROOT / "exports"


def main() -> int:
    problems = []

    # 1. all three parts print as single watertight solids
    for name in ("camlock_tool_side", "camlock_cam_ring", "camlock_arm_side"):
        subprocess.run([sys.executable, str(ROOT / "parts" / f"{name}.py")], cwd=ROOT,
                       check=True, stdout=subprocess.DEVNULL)
        m = trimesh.load(str(EXPORTS / f"{name}.stl"))
        if not m.is_watertight or len(m.split(only_watertight=False)) != 1:
            problems.append(f"{name}: not a single watertight solid "
                            f"(bodies={len(m.split(only_watertight=False))}, wt={m.is_watertight})")

    # 2. locator is EQUAL 120deg (symmetric)
    a = sorted(x % 360 for x in geo.FEATURE_ANGLES)
    gaps = [(a[(i + 1) % 3] - a[i]) % 360 for i in range(3)]
    if max(gaps) - min(gaps) > 1.0:
        problems.append(f"locator not equal 120deg: gaps {gaps}")

    # 3. interrupted thread engages AND inserts
    if geo.R_CATCH_IN >= geo.R_LUG:
        problems.append(f"cam catch r{geo.R_CATCH_IN} doesn't reach inside lug OD r{geo.R_LUG} (no grab)")
    if geo.GAP_BORE_R < geo.R_LUG:
        problems.append(f"gap bore r{geo.GAP_BORE_R} < lug OD r{geo.R_LUG} — lugs can't insert")
    gap_arc = 120.0 - geo.CATCH_ARC
    if geo.LUG_ARC >= gap_arc:
        problems.append(f"lug arc {geo.LUG_ARC} >= gap arc {gap_arc} — lug won't pass the gap")
    if geo.FEATURE_R - geo.VEE_L / 2 <= geo.ARM_BORE_D / 2:
        problems.append("vees overlap the central cam bore")

    # 4. known-physics on the statics --------------------------------------------------
    sl = cs.self_locking()
    if not sl["self_locking"]:
        problems.append(f"thread NOT self-locking (lead {sl['lead_deg']} >= friction "
                        f"{sl['friction_deg']} deg) — would back-drive")

    # anti-spin margin is geometry-set: identical at two different preloads
    m1, m2 = cs.antispin(40.0)["margin"], cs.antispin(120.0)["margin"]
    if abs(m1 - m2) > 0.05:
        problems.append(f"anti-spin margin should be preload-independent but {m1} != {m2}")
    if cs.antispin(cs.TARGET_PRELOAD)["margin"] < cs.SAFETY:
        problems.append(f"cam locking torque spins the tool (anti-spin x"
                        f"{cs.antispin(cs.TARGET_PRELOAD)['margin']})")

    # preload linear in servo torque
    if abs(cs.preload_from_servo(2.0) - 2 * cs.preload_from_servo(1.0)) > 1e-6:
        problems.append("preload should be linear in servo torque")

    # design holds the service load AND beats the Fidlock registration margin
    L = fid.PRINTER_TOOL
    cam = cs.evaluate(cs.TARGET_PRELOAD, L)
    fdk = fid.evaluate(fid.Retention("fidlock", fidlock=True, rim_lock=False), L)
    if not cam["pass"]:
        problems.append(f"camlock should hold the service load but FAILED (binds {cam['binding']})")
    if cam["margins"]["moment_liftoff"] <= fdk["margins"]["moment_liftoff"]:
        problems.append("camlock should beat the Fidlock liftoff margin (its reason to add a servo)")

    # report ---------------------------------------------------------------------------
    a_s = cs.antispin(cs.TARGET_PRELOAD)
    print(f"camlock: Ø{geo.PLATE_OD:.0f}mm, 120deg locator (r{geo.FEATURE_R:.0f}), 3-start ACME "
          f"draw-lock (dm{geo.POST_SHAFT_D:.0f}, lead{geo.LEAD:.0f}mm), servo drive arm")
    print(f"  self-lock x{sl['margin']} | anti-spin x{a_s['margin']} (geometry-set) | "
          f"target {cs.TARGET_PRELOAD:.0f}N preload from {cs.servo_for_preload(cs.TARGET_PRELOAD):.2f} N.m")
    print(f"  vs service moment {L.moment_Nm()*1000:.0f}Nmm: liftoff x{cam['margins']['moment_liftoff']:.1f} "
          f"(Fidlock x{fdk['margins']['moment_liftoff']:.1f}), repeatability ~{cam['repeatability_um']}um "
          f"[PREDICTION until bench test]")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: draw-lock inserts + grabs, self-locks, holds its tool without spinning it, and "
          "seats it stiffer than the Fidlock (at the cost of a servo)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
