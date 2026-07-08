#!/usr/bin/env python3
"""Gate: the reverse-engineered omni wheel assembles, the rollers spin free, AND ground
contact is continuous (two staggered rows cover each other's gaps).

Grey-box reverse engineering validated by the interference checker (sim/interference.py):
  - both printed parts are single watertight solids; barrel geometry self-consistent;
  - 2*N rollers (two rows) on the hub blend into a continuous OD at R_EFF;
  - ZERO solid interference — rollers clear the hub, their own row, AND the other row;
  - CONTINUITY: each roller's angular coverage >= 90/N deg, so the staggered rows leave no
    gap in ground contact (a single row would bump every rotation);
  - the roller bore accepts the off-the-shelf metal pin.
"""

import subprocess
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "parts"))
import interference as ix  # noqa: E402
import _omni as o  # noqa: E402

BUILD = ROOT / "build"


def roller_xform(i, row):
    (cx, cy, cz), deg = o.roller_center(i, row)
    return (trimesh.transformations.translation_matrix([cx, cy, cz])
            @ trimesh.transformations.rotation_matrix(np.radians(deg), [0, 0, 1])
            @ trimesh.transformations.rotation_matrix(np.radians(90), [1, 0, 0]))


def pin_retention(hub):
    """Probe the pin pocket: outboard of the pin the entry throat must be NARROWER than the
    pin (the lips that snap over it). Returns the max grip (pin_width - throat) found, in mm."""
    (cx, cy, cz), deg = o.roller_center(0, 0)
    a = np.radians(deg)
    tang = np.array([-np.sin(a), np.cos(a), 0.]); rad = np.array([np.cos(a), np.sin(a), 0.])
    Pc = np.array([cx, cy, cz]); s = o.HALF_L + 2.0
    best = -9.9
    for dr in (0.25, 0.5, 0.75):
        dzs = np.linspace(-3, 3, 241)
        inside = hub.contains(np.array([Pc + tang * s + rad * dr + np.array([0, 0, d]) for d in dzs]))
        openz = dzs[~inside]
        gap = (openz.max() - openz.min()) if len(openz) else 0.0
        pin_w = 2 * np.sqrt(max(0.0, (o.PIN_D / 2) ** 2 - dr ** 2))
        best = max(best, pin_w - gap)          # >0 means the throat grips the pin here
    return best


def main() -> int:
    problems = []
    for name in ("omni_roller", "omni_hub"):
        subprocess.run([sys.executable, str(ROOT / "parts" / f"{name}.py")], cwd=ROOT,
                       check=True, stdout=subprocess.DEVNULL)

    if o.validity():
        problems += [f"geometry: {w}" for w in o.validity()]
    if o.ROLLER_BORE <= o.PIN_D:
        problems.append(f"roller bore {o.ROLLER_BORE} must exceed pin {o.PIN_D}")

    cont = o.continuity()
    if not cont["continuous"]:
        problems.append(f"contact NOT continuous: {cont}")

    # snap-fit retention: throat narrower than the pin (snaps + retains), seat >= pin
    if o.PIN_SNAP_MOUTH >= o.PIN_D:
        problems.append(f"snap throat {o.PIN_SNAP_MOUTH} must be < pin {o.PIN_D} to retain it")
    if o.HUB_PIN_BORE < o.PIN_D:
        problems.append(f"hub pin seat {o.HUB_PIN_BORE} must be >= pin {o.PIN_D}")

    hub = trimesh.load(str(BUILD / "omni_hub.stl"))
    roller = trimesh.load(str(BUILD / "omni_roller.stl"))
    for nm, m in (("hub", hub), ("roller", roller)):
        if not m.is_watertight or len(m.split(only_watertight=False)) != 1:
            problems.append(f"{nm} not a single watertight solid")

    sc = ix.Scene().place("hub", hub, np.eye(4))
    maxr = 0.0
    for row in range(o.ROWS):
        for i in range(o.N_ROLLERS):
            T = roller_xform(i, row)
            sc.place(f"r{row}_{i}", roller, T)
            v = trimesh.transform_points(roller.vertices, T)
            maxr = max(maxr, float(np.max(np.hypot(v[:, 0], v[:, 1]))))
    if abs(maxr - o.R_EFF) > 0.4:
        problems.append(f"assembled OD radius {maxr:.2f} != R_EFF {o.R_EFF}")

    inter = sc.interferences()
    if inter:
        problems.append(f"assembly interferes (rollers can't spin): "
                        f"{[(h['pair'], h['volume_mm3']) for h in inter[:4]]}")

    grip = pin_retention(hub)                  # the throat must actually snap over the pin
    if grip < 0.15:
        problems.append(f"pin NOT retained: throat wider than the pin outboard (grip {grip:.2f}mm)")

    w = (hub.bounds[1] - hub.bounds[0]).round(1).tolist()
    print(f"omni wheel: {o.ROWS} rows x {o.N_ROLLERS} rollers, OD {2*maxr:.0f}mm, barrel "
          f"{2*o.BARREL_MAX:.0f}mm, width {w[2]:.0f}mm, pin {o.PIN_D}mm")
    print(f"  continuity: coverage {cont['coverage_deg']}deg >= need {cont['need_deg']}deg "
          f"(margin {cont['margin']}x) -> rolls without bumping")
    print(f"  pin: Ø{o.PIN_D}mm, snaps past a {o.PIN_SNAP_MOUTH}mm throat into a {o.HUB_PIN_BORE}mm "
          f"seat, lips grip its outboard face by {grip:.2f}mm")
    print(f"  hub {w}  roller {(roller.bounds[1]-roller.bounds[0]).round(1).tolist()}  (watertight)")
    print(f"  assembly interference: {len(inter)} (0 = all {o.ROWS*o.N_ROLLERS} rollers spin free)")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: two-row omni wheel — rollers spin free, OD = R_EFF, ground contact continuous")
    return 0


if __name__ == "__main__":
    sys.exit(main())
