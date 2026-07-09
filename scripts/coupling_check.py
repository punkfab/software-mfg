#!/usr/bin/env python3
"""Gate: the tool-changer coupling registers in ONE orientation, prints as clean solids,
AND the statics say it holds the tool at the worst prying pose.

Ties parts/_coupling.py (geometry) to sim/coupling_statics.py (load model), the same way
omni_check ties the omni wheel CAD to the interference checker:

  - both faces are single watertight solids (printable);
  - the 3 vees are spaced UNEQUALLY -> the tool seats in exactly one clocking (keys the
    single pogo set + the jaw direction — no duplicated pins);
  - known-physics on the statics: an on-axis pull with no offset makes ZERO moment; a rim
    tie (lug at RIM_R) carries MORE moment than a central barb for the same force (the
    whole reason to move load to the circumference); more preload -> tighter repeatability;
  - the design conclusion holds: a bare magnet coupler FAILS the service moment, the
    kinematic+Fidlock coupling PASSES (the rim lock is a margin upgrade, not a requirement).
"""

import subprocess
import sys
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "parts"))
import _coupling as geo  # noqa: E402
import coupling_statics as cs  # noqa: E402

EXPORTS = ROOT / "exports"


def unequal_spacing(angles):
    """Sorted angular gaps around the circle; keyed iff they are NOT all equal."""
    a = sorted(x % 360 for x in angles)
    gaps = [(a[(i + 1) % len(a)] - a[i]) % 360 for i in range(len(a))]
    return gaps, max(gaps) - min(gaps) > 5.0     # >5deg spread -> single orientation


def main() -> int:
    problems = []

    # 1. both faces build as single watertight solids
    for name in ("coupling_arm_side", "coupling_tool_side"):
        subprocess.run([sys.executable, str(ROOT / "parts" / f"{name}.py")], cwd=ROOT,
                       check=True, stdout=subprocess.DEVNULL)
        m = trimesh.load(str(EXPORTS / f"{name}.stl"))
        if not m.is_watertight or len(m.split(only_watertight=False)) != 1:
            problems.append(f"{name}: not a single watertight solid "
                            f"(bodies={len(m.split(only_watertight=False))}, wt={m.is_watertight})")

    # 2. registration keys to ONE orientation (unequal vee spacing)
    gaps, keyed = unequal_spacing(geo.FEATURE_ANGLES)
    if not keyed:
        problems.append(f"vees equally spaced {gaps} -> seats {len(geo.FEATURE_ANGLES)} ways; "
                        "unequal spacing needed to key a single orientation")
    # vees must clear the mount bolts and the rim ring
    if geo.FEATURE_R + geo.VEE_L / 2 > geo.CATCH_RING_IR + 0.01:
        problems.append("vee outer end crosses into the rim catch ring")

    # 3. known-physics on the statics ---------------------------------------------------
    ret = cs.Retention("kinematic + Fidlock", fidlock=True, rim_lock=False)
    ret_rim = cs.Retention("+ rim lock", fidlock=True, rim_lock=True)

    # (a) on-axis pull, no offset, no process force -> zero prying moment
    axial_only = cs.ServiceLoad(tool_mass_kg=0.2, cg_offset_m=0.05, axis_horizontal=False,
                                proc_force_N=0.0, proc_lever_m=0.0)
    if axial_only.moment_Nm() > 1e-9:
        problems.append(f"on-axis load makes a phantom moment {axial_only.moment_Nm()} (should be 0)")

    # (b) a rim tie carries MORE moment than a central barb for the same force (radius leverage)
    if ret_rim.m_ultimate_Nm() <= ret.m_ultimate_Nm():
        problems.append("rim lock did not raise ultimate moment capacity — radius leverage lost")

    # (c) more preload -> tighter (smaller) repeatability number
    if cs.repeatability_um(ret_rim) >= cs.repeatability_um(ret):
        problems.append("higher preload did not improve repeatability")

    # (d) the design conclusion: bare magnet FAILS the service moment, kinematic+Fidlock PASSES
    L = cs.PRINTER_TOOL
    bare = cs.evaluate(cs.Retention("magnets-only", fidlock=False, rim_lock=False), L)
    good = cs.evaluate(ret, L)
    if bare["pass"]:
        problems.append("a bare magnet coupler should FAIL the prying moment but passed")
    if not good["pass"]:
        problems.append(f"kinematic+Fidlock should hold the service load but FAILED "
                        f"(binds {good['binding']} x{good['binding_margin']})")

    # report ----------------------------------------------------------------------------
    print(f"coupling: Ø{geo.PLATE_OD:.0f}mm, 3 vees at {geo.FEATURE_ANGLES} (gaps {gaps} -> "
          f"single orientation), rim lugs at r{geo.RIM_R:.0f}, one keyed pogo set")
    print(f"  service moment {L.moment_Nm()*1000:.0f}Nmm  ->  Fidlock liftoff "
          f"x{good['margins']['moment_liftoff']:.1f} (PASS), rim-lock liftoff "
          f"x{cs.evaluate(ret_rim, L)['margins']['moment_liftoff']:.1f}")
    print(f"  repeatability ~{good['repeatability_um']}um (Fidlock) -> "
          f"~{cs.evaluate(ret_rim, L)['repeatability_um']}um (+rim) [PREDICTION until bench test]")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: coupling registers one way, prints clean, and the Fidlock catch secures the "
          "tool at the worst prying pose (rim lock = margin upgrade)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
