"""omni_kinematics.py — holonomic kinematics for a 3-omni-wheel base (body twist <-> wheel speeds).

The ONE algorithm that maps a desired body motion (vx, vy, yaw-rate) to the three omni-wheel drive
speeds, and back (odometry). Shared by BOTH the MuJoCo sim (sim/lekiwi_sim.py) and the real hardware
driver (bridge/omni_drive.py), so the model you tune in sim and the robot you drive can never drift
apart — the sim-is-a-cache-of-reality rule, made literal.

Frame (REP-103): body +x FORWARD, +y LEFT, +z UP, yaw wz CCW-positive. Wheel i sits at azimuth
phi_i (its position angle from +x, CCW) at radius base_r from centre; its rolling/drive direction is
the CCW TANGENT there, d_i = (-sin phi_i, cos phi_i). A point on the wheel moves with the body as
(vx, vy) + wz x r_i, and only the component along d_i is DRIVEN (the passive side-rollers absorb the
rest — that's what makes it holonomic). Projecting onto d_i:

    v_i     = -sin(phi_i)*vx + cos(phi_i)*vy + base_r*wz     # wheel-rim linear speed (m/s)
    omega_i = spin_i * v_i / wheel_r                         # wheel angular speed (rad/s)

`spin_i` (+/-1) flips a wheel whose motor/horn is mounted mirrored, so a commanded "forward" really
goes forward — set it once per build. `base_r` and the `phi_i` come from the REAL base plate: MEASURE
them. `wheel_r` is the Ø70 custom omni wheel (parts/_omni.py R_EFF=35 -> 35 mm).

    python bridge/omni_kinematics.py --selftest      # round-trip + known-motion asserts (headless)
    from bridge.omni_kinematics import OmniGeometry, body_to_wheels, wheels_to_raw
"""
import math
from dataclasses import dataclass

try:
    import numpy as np
except ImportError:                          # forward kinematics (odometry) needs numpy; inverse doesn't
    np = None

RESOLUTION = 4096            # STS3215 counts/rev (matches sim/feetech_protocol.RESOLUTION)


@dataclass(frozen=True)
class OmniGeometry:
    """The three numbers the kinematics needs, all from the real base. Defaults = Dan's build."""
    wheel_r_mm: float = 35.0                          # Ø70 omni wheel (parts/_omni.py R_EFF). MEASURE.
    base_r_mm: float = 72.746                          # centre -> wheel CONTACT radius (see below).
    azimuths_deg: tuple = (0.0, 120.0, 240.0)         # 3 spokes at i*120° (kiwi-plate-manual-countersink.scad)
    spin: tuple = (1.0, 1.0, 1.0)                     # per-wheel motor-direction sign (mounting reality)
    # base_r is the perpendicular distance from the base centre to each wheel's rolling line (the yaw
    # moment arm) = the wheel-contact radius. MEASURED from the assembled base: the wheel-centre circle
    # has an inscribed equilateral triangle whose side (chord between adjacent centres, 120° apart) is
    # 126 mm, so base_r = chord/√3 = 126/√3 = 72.746 mm. +x is along spoke 0 (front, wheel 16).

    @property
    def wheel_r(self):
        return self.wheel_r_mm / 1000.0              # m

    @property
    def base_r(self):
        return self.base_r_mm / 1000.0              # m

    def matrix(self):
        """Inverse-kinematic rows: rim linear speed v_i = row_i . (vx, vy, wz)."""
        return [(-math.sin(math.radians(a)), math.cos(math.radians(a)), self.base_r)
                for a in self.azimuths_deg]


def body_to_wheels(geo: OmniGeometry, vx, vy, wz):
    """Body twist (vx, vy in m/s; wz in rad/s) -> (w0, w1, w2) wheel angular speeds (rad/s).
    This is the inverse kinematics you command the motors with."""
    out = []
    for row, s in zip(geo.matrix(), geo.spin):
        v = row[0] * vx + row[1] * vy + row[2] * wz
        out.append(s * v / geo.wheel_r)
    return tuple(out)


def wheels_to_body(geo: OmniGeometry, w0, w1, w2):
    """(w0, w1, w2) wheel speeds (rad/s) -> body twist (vx, vy, wz). Odometry: invert the 3x3."""
    if np is None:
        raise RuntimeError("wheels_to_body (odometry) needs numpy; only body_to_wheels is numpy-free")
    M = np.array(geo.matrix())
    v = np.array([geo.spin[i] * w * geo.wheel_r for i, w in enumerate((w0, w1, w2))])
    vx, vy, wz = np.linalg.solve(M, v)               # 3 wheels -> square, invertible (non-degenerate)
    return float(vx), float(vy), float(wz)


def rad_s_to_raw(w):
    """One wheel speed (rad/s) -> STS3215 Goal_Velocity raw (signed counts/s), no clamp."""
    return w * RESOLUTION / (2 * math.pi)


def wheels_to_raw(wheels_rad_s, max_raw=4000):
    """(w0, w1, w2) rad/s -> STS3215 Goal_Velocity raws. If any wheel exceeds `max_raw`, scale ALL
    THREE down together so the motion DIRECTION is preserved (a holonomic move stays straight, just
    slower) rather than clipping one wheel and skewing the path."""
    raw = [rad_s_to_raw(w) for w in wheels_rad_s]
    peak = max((abs(r) for r in raw), default=0.0)
    if peak > max_raw:
        raw = [r * max_raw / peak for r in raw]
    return tuple(int(round(r)) for r in raw)


def body_to_raw(geo: OmniGeometry, vx, vy, wz, max_raw=4000):
    """The full command path: body twist -> wheel rad/s -> clamped STS3215 Goal_Velocity raws."""
    return wheels_to_raw(body_to_wheels(geo, vx, vy, wz), max_raw)


def selftest():
    geo = OmniGeometry()
    ok = True

    # pure rotation: every wheel spins the same (rim speed = base_r*wz for all)
    w = body_to_wheels(geo, 0.0, 0.0, 1.0)
    if not all(abs(wi - w[0]) < 1e-9 for wi in w):
        print("FAIL pure-rotation not equal:", w); ok = False

    # pure translation on a 120°-symmetric base: the three wheel speeds sum to ~0 (no net spin ->
    # no yaw), for any translation direction (azimuth-phase-independent)
    for vx, vy in [(0.5, 0.0), (0.0, 0.3), (0.2, -0.15)]:
        if abs(sum(body_to_wheels(geo, vx, vy, 0.0))) > 1e-9:
            print(f"FAIL pure-translation ({vx},{vy}) wheels don't sum to 0"); ok = False

    # inverse->forward round-trip (odometry) for several twists
    if np is not None:
        for tw in [(0.20, 0.0, 0.0), (0.0, 0.15, 0.0), (0.0, 0.0, 0.5), (0.12, -0.06, 0.30)]:
            back = wheels_to_body(geo, *body_to_wheels(geo, *tw))
            if not all(abs(a - b) < 1e-6 for a, b in zip(tw, back)):
                print(f"FAIL round-trip {tw} -> {back}"); ok = False
    else:
        print("  (numpy absent: skipped odometry round-trip)")

    # raw clamp preserves direction (ratios) when scaling down
    ws = body_to_wheels(geo, 3.0, 0.0, 0.0)          # deliberately too fast
    raw = wheels_to_raw(ws, max_raw=1000)
    if max(abs(r) for r in raw) > 1000:
        print("FAIL clamp exceeded max_raw:", raw); ok = False
    ratio_ws = ws[0] / ws[2] if ws[2] else 0
    ratio_raw = raw[0] / raw[2] if raw[2] else 0
    if abs(ratio_ws - ratio_raw) > 1e-3:
        print("FAIL clamp changed direction ratio:", ratio_ws, ratio_raw); ok = False

    if ok:
        print(f"omni_kinematics selftest OK  (wheel_r={geo.wheel_r_mm}mm base_r={geo.base_r_mm}mm "
              f"azimuths={geo.azimuths_deg})")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(selftest())
