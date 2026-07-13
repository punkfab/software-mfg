"""omni_drive.py — drive the 3-omni holonomic base over the Feetech bus, and provision its wheel IDs.

Ties the shared kinematics (bridge/omni_kinematics.py) to the real serial bus (bridge/feetech_bus.py):
a body twist (vx forward, vy left, wz yaw) becomes three STS3215 wheel-velocity commands.

ID PLAN: the wheel motors take IDs 16, 17, 18 — leaving 0..15 for the SO-101 arm axes, effectors,
and the tool-changer, all on the same bus. `--set-ids` provisions them one at a time (factory servos
ship as ID 1, so they must be connected singly to avoid a collision).

    python bridge/omni_drive.py --scan                      # list ids on the bus
    python bridge/omni_drive.py --set-ids                   # guided: assign 16,17,18 + wheel mode
    python bridge/omni_drive.py --test figure8              # scripted motions (forward/strafe/spin/box/figure8)
    python bridge/omni_drive.py --teleop                    # WASD drive (Q/E rotate, space stop)
    python bridge/omni_drive.py --test all --dry-run        # no hardware: print the wheel commands
    --port /dev/ttyUSB0   --ids 16,17,18   --max-raw 4000   --speed 0.15   (defaults shown)
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bridge.omni_kinematics import OmniGeometry, body_to_wheels, wheels_to_raw, wheels_to_body
from bridge.feetech_bus import FeetechBus, FeetechError, DryBus

WHEEL_IDS = (16, 17, 18)              # 0..15 reserved for arm / effectors / tool changer
# Calibrated build (2026-07-12): daisy-chain order 16 -> 17 -> 18; all spin CW on +command (spin
# 1,1,1). FRONT is over wheel 16, so azimuths (0,120,240) map 16->0° (front), 17->120°, 18->240°.
# Forward/back is correct by construction (16 lies on the +x axis, idle for pure forward); strafe &
# yaw handedness depend on whether 17 is the rear-LEFT (120°) or rear-RIGHT wheel — swap 17<->18 if
# a left command goes right. base_r measured = 72.746 mm (126 mm wheel-centre chord / √3).


def open_bus(args):
    if args.dry_run:
        return DryBus()
    return FeetechBus(args.port)


def send_twist(bus, geo, ids, vx, vy, wz, max_raw):
    raw = wheels_to_raw(body_to_wheels(geo, vx, vy, wz), max_raw)
    bus.sync_velocity(dict(zip(ids, raw)))
    return raw


# --- ID provisioning ------------------------------------------------------------------------
def set_ids(bus, ids):
    """Guided: connect ONE wheel at a time; assign each the next target ID + put it in wheel mode."""
    print(f"Provisioning wheel IDs {list(ids)} (0..15 stay free for arm/effectors/toolchanger).")
    for target in ids:
        input(f"\n  → Connect ONLY the wheel to become ID {target}, power the bus, press Enter... ")
        present = bus.scan()
        if present == [target]:
            print(f"    already ID {target}.")
        elif len(present) == 1:
            found = present[0]
            print(f"    found ID {found} → setting to {target} ...")
            bus.set_id(found, target)
        elif not present:
            print("    ✗ no servo answered — check power/wiring; skipping.")
            continue
        else:
            print(f"    ✗ multiple servos on the bus {present} — connect ONE at a time; skipping.")
            continue
        bus.set_wheel_mode(target, wheel=True)
        bus.set_velocity(target, 0)
        print(f"    ID {target}: wheel mode set, stopped. ✓")
    print("\nDone. Re-run with --scan to confirm, or --test to drive.")


# --- scripted test patterns -----------------------------------------------------------------
def _segments(pattern, v, w):
    """(label, vx, vy, wz, seconds) segments for a named pattern."""
    P = {
        "forward":   [("forward", v, 0, 0, 2)],
        "strafe":    [("strafe-left", 0, v, 0, 2), ("strafe-right", 0, -v, 0, 2)],
        "rotate":    [("spin-ccw", 0, 0, w, 2), ("spin-cw", 0, 0, -w, 2)],
        "box":       [("+x", v, 0, 0, 1.5), ("+y", 0, v, 0, 1.5),
                      ("-x", -v, 0, 0, 1.5), ("-y", 0, -v, 0, 1.5)],
        "figure8":   [("fwd+ccw", v, 0, w, 2.5), ("fwd+cw", v, 0, -w, 2.5)],
    }
    if pattern == "all":
        seq = []
        for k in ("forward", "strafe", "rotate", "box", "figure8"):
            seq += P[k]
        return seq
    return P[pattern]


def run_test(bus, geo, ids, pattern, v, w, max_raw, dt=0.02):
    segs = _segments(pattern, v, w)
    px = py = pth = 0.0                       # open-loop odometry estimate (integrate the command)
    for i in ids:
        bus.torque(i, True)                   # wheel mode won't drive with torque disabled
    try:
        for label, vx, vy, wz, secs in segs:
            print(f"  {label:12} v=({vx:+.2f},{vy:+.2f}) w={wz:+.2f}  {secs:.1f}s")
            n = int(secs / dt)
            for _ in range(n):
                send_twist(bus, geo, ids, vx, vy, wz, max_raw)
                px += vx * dt; py += vy * dt; pth += wz * dt      # world-ish, no rotation compounding
                time.sleep(dt)
            bus.stop_all(ids)
            time.sleep(0.25)
    finally:
        bus.stop_all(ids)
        for i in ids:
            bus.torque(i, False)
    print(f"  commanded net displacement ≈ x{px:+.2f} y{py:+.2f} m, yaw{pth:+.2f} rad (open-loop).")


def run_spin(bus, wid, raw):
    """Spin ONE wheel continuously at `raw` (counts/s, sign = direction) until Ctrl-C — for
    identifying which physical wheel an ID is and which way + turns. Prints live encoder/speed."""
    print(f"spinning id {wid} at raw {raw:+d} (sign=direction). Ctrl-C to stop.")
    try:
        bus.torque(wid, True)
        bus.set_velocity(wid, raw)
        while True:
            time.sleep(0.3)
            pos = bus.read(wid, "Present_Position")
            vel = bus.read(wid, "Present_Velocity", signmag=True)
            print(f"\r  id {wid}  pos={pos:5d}  present_vel={vel:+5d}   ", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        bus.set_velocity(wid, 0)
        bus.torque(wid, False)
        print("\nstopped.")


# --- keyboard teleop (Unix raw-mode, stdlib only) -------------------------------------------
def _autorepeat(fast):
    """Speed up (or restore) the terminal key auto-repeat so a HELD key streams smoothly instead of
    stuttering — best-effort via xset when an X display is present; a no-op otherwise."""
    import os
    import shutil
    import subprocess
    if not (shutil.which("xset") and os.environ.get("DISPLAY")):
        return
    try:
        args = ["xset", "r", "rate", "180", "40"] if fast else ["xset", "r", "rate"]
        subprocess.run(args, check=False, timeout=1,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def run_teleop(bus, geo, ids, v, w, max_raw, hold_timeout=0.3, dt=0.02):
    """DEAD-MAN drive: the base moves ONLY while a WASD/QE key is held, and stops the moment you let
    go. A held key auto-repeats, refreshing a keep-alive deadline; when the repeats stop (key released),
    the deadline lapses within `hold_timeout` and the twist is zeroed. (One key at a time — the terminal
    only repeats the most-recent key, so no diagonals in this mode.)"""
    import select
    import termios
    import time as _t
    import tty
    MOVE = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0)}   # ×speed
    ROT = {"q": 1, "e": -1}                                                     # ×(speed/base_r)
    print("Teleop (HOLD to move, release = stop):  W/S fwd/back  A/D strafe  Q/E rotate  "
          "+/- speed  X quit")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    speed = v
    twist = (0.0, 0.0, 0.0)
    deadline = 0.0
    try:
        _autorepeat(fast=True)
        tty.setcbreak(fd)
        for i in ids:
            bus.torque(i, True)
        while True:
            r, _, _ = select.select([sys.stdin], [], [], dt)
            now = _t.monotonic()
            if r:
                c = sys.stdin.read(1).lower()
                if c in ("x", "\x1b"):
                    break
                elif c == "+":
                    speed = min(speed * 1.25, 1.0)
                elif c == "-":
                    speed = max(speed * 0.8, 0.02)
                elif c in MOVE:
                    mx, my, _mw = MOVE[c]
                    twist = (mx * speed, my * speed, 0.0)
                    deadline = now + hold_timeout                 # refreshed by every auto-repeat
                elif c in ROT:
                    twist = (0.0, 0.0, ROT[c] * speed / max(geo.base_r, 1e-3))
                    deadline = now + hold_timeout
                # any other key (incl. space) just lets the deadline lapse -> stop
            if now >= deadline:                                  # released / no key -> DEAD-MAN stop
                twist = (0.0, 0.0, 0.0)
            raw = send_twist(bus, geo, ids, *twist, max_raw)
            held = now < deadline
            print(f"\r {'MOVE' if held else 'stop'}  v=({twist[0]:+.2f},{twist[1]:+.2f}) "
                  f"w={twist[2]:+.2f}  raw={raw}  speed={speed:.2f}   ", end="", flush=True)
    finally:
        _autorepeat(fast=False)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.stop_all(ids)
        for i in ids:
            bus.torque(i, False)
        print("\nstopped.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--ids", default=",".join(map(str, WHEEL_IDS)),
                    help="wheel servo ids in mount order (azimuths 60,180,300)")
    ap.add_argument("--dry-run", action="store_true", help="no hardware — print wheel commands")
    ap.add_argument("--scan", action="store_true")
    ap.add_argument("--set-ids", action="store_true", help="guided provisioning to the --ids + wheel mode")
    ap.add_argument("--test", metavar="PATTERN", nargs="?", const="all",
                    choices=["forward", "strafe", "rotate", "box", "figure8", "all"])
    ap.add_argument("--teleop", action="store_true")
    ap.add_argument("--turn", type=int, metavar="ID", help="spin ONE wheel until Ctrl-C (identify)")
    ap.add_argument("--turn-raw", type=int, default=1500, help="raw speed for --turn (negative = reverse)")
    ap.add_argument("--speed", type=float, default=0.15, help="linear speed m/s")
    ap.add_argument("--omega", type=float, default=1.0, help="yaw rate rad/s")
    ap.add_argument("--max-raw", type=int, default=4000, help="Goal_Velocity clamp (counts/s)")
    ap.add_argument("--hold-timeout", type=float, default=0.3,
                    help="dead-man: stop this many seconds after the last key repeat (release)")
    ap.add_argument("--wheel-r", type=float, default=35.0, help="wheel radius mm (Ø70 -> 35)")
    ap.add_argument("--base-r", type=float, default=72.746,
                    help="centre->wheel radius mm (measured: 126mm chord / √3)")
    ap.add_argument("--azimuths", default="0,120,240", help="wheel azimuths deg, CCW from +x (plate spokes)")
    ap.add_argument("--spin", default="1,1,1", help="per-wheel direction sign (mounting)")
    args = ap.parse_args()

    ids = [int(x) for x in args.ids.split(",")]
    geo = OmniGeometry(wheel_r_mm=args.wheel_r, base_r_mm=args.base_r,
                       azimuths_deg=tuple(float(a) for a in args.azimuths.split(",")),
                       spin=tuple(float(s) for s in args.spin.split(",")))

    try:
        bus = open_bus(args)
    except FeetechError as e:
        print("bus error:", e); return 2

    try:
        if args.set_ids:
            set_ids(bus, ids)
        elif args.scan:
            print("ids on bus:", bus.scan())
        elif args.test:
            print(f"test '{args.test}'  geo(wheel_r={geo.wheel_r_mm} base_r={geo.base_r_mm} "
                  f"az={geo.azimuths_deg})  ids={ids}")
            run_test(bus, geo, ids, args.test, args.speed, args.omega, args.max_raw)
        elif args.turn is not None:
            run_spin(bus, args.turn, args.turn_raw)
        elif args.teleop:
            run_teleop(bus, geo, ids, args.speed, args.omega, args.max_raw,
                       hold_timeout=args.hold_timeout)
        else:
            print(__doc__)
    finally:
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
