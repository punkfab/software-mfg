#!/usr/bin/env python3
"""Gate: the 3-omni holonomic drive — kinematics + Feetech command path (no hardware).

Asserts the body<->wheel kinematics (odometry round-trip; the three canonical motions), that the
raw clamp preserves motion direction, and that every wheel command survives the STS3215 codec
(sign-magnitude Goal_Velocity) intact. Hardware-free, so it runs in `make check`.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))

import bridge.omni_kinematics as K  # noqa: E402
from bridge.omni_kinematics import (OmniGeometry, body_to_wheels, wheels_to_body,  # noqa: E402
                                    body_to_raw, wheels_to_raw, rad_s_to_raw)
import feetech_protocol as fp  # noqa: E402


def main() -> int:
    problems = []
    geo = OmniGeometry()

    # 1. the kinematics module's own selftest (round-trip, canonical motions, clamp)
    if K.selftest() != 0:
        problems.append("omni_kinematics selftest failed")

    # 2. canonical motions: pure rotation -> all wheels equal; pure translation on a 120°-symmetric
    # base -> the three wheel speeds sum to ~0 (no net yaw), for any translation direction
    if len({round(w, 6) for w in body_to_wheels(geo, 0, 0, 1.0)}) != 1:
        problems.append("pure rotation should spin all three wheels equally")
    if abs(sum(body_to_wheels(geo, 0.2, -0.1, 0))) > 1e-9:
        problems.append("pure translation should sum to zero wheel speed (no yaw)")

    # 3. odometry round-trip (inverse then forward reproduces the twist)
    tw = (0.12, -0.06, 0.4)
    if any(abs(a - b) > 1e-6 for a, b in zip(tw, wheels_to_body(geo, *body_to_wheels(geo, *tw)))):
        problems.append("odometry round-trip mismatch")

    # 4. the command path survives the STS3215 codec (2-byte sign-magnitude), both signs
    for raw in body_to_raw(geo, 0.15, 0.1, 0.5, max_raw=4000):
        if fp.from_le(fp.le(raw, 2), signmag=True) != raw:
            problems.append(f"Goal_Velocity codec changed {raw}")

    # 5. clamp bounds the command AND scales all wheels proportionally (holonomic path stays straight)
    ws = body_to_wheels(geo, 5.0, 0, 0)                       # deliberately too fast
    unclamped = [rad_s_to_raw(w) for w in ws]
    raw = wheels_to_raw(ws, max_raw=3000)
    if max(abs(r) for r in raw) > 3000:
        problems.append("raw clamp exceeded max_raw")
    ratios = [r / u for r, u in zip(raw, unclamped) if abs(u) > 1e-6]
    if ratios and (max(ratios) - min(ratios)) > 1e-2:        # one common scale -> direction preserved
        problems.append("clamp changed direction (non-proportional scale)")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: 3-omni kinematics + Feetech command path verified (hardware-free)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
