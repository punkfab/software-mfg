"""servo_read.py — read a Feetech STS3215's shaft position as a SENSOR (a knob / joint encoder).

Torque is disabled so the shaft turns freely by hand, and Present_Position (0..4095 = 0..360°,
single-turn absolute) is polled and printed live. Reuses bridge/feetech_bus (the sim-validated codec),
so it shares the bus with the wheel servos (16/17/18) and the arm — just address a different id.

    python bridge/servo_read.py --id 6            # live absolute angle, Ctrl-C to stop
    python bridge/servo_read.py --id 6 --zero     # angle RELATIVE to where it started (a re-zeroable knob)
    python bridge/servo_read.py --scan            # list ids on the bus
    --port /dev/ttyACM0   --hz 20   --hold-torque (keep it driven, don't free the shaft)
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from bridge.feetech_bus import FeetechBus, FeetechError

RES = 4096                       # STS3215 counts/rev (single-turn absolute)


def read(bus, wid):
    pos = bus.read(wid, "Present_Position")
    return pos, pos * 360.0 / RES


def load(bus, wid):
    """Present_Load as a torque/effort PROXY (motor current, not calibrated N·m). bit 10 = direction,
    bits 0-9 = magnitude 0-1000. Only meaningful with torque ENABLED (--hold-torque); a freed shaft
    draws ~no current, so load reads ~0 no matter how hard you push."""
    raw = bus.read(wid, "Present_Load")
    return ("-" if raw & 0x400 else "+"), raw & 0x3FF


def run(port, wid, hz, zero, free=True):
    with FeetechBus(port) as bus:
        if free:
            bus.torque(wid, False)                      # release so it turns freely by hand
        p0, _ = read(bus, wid)
        print(f"reading id {wid} on {port} "
              f"({'torque off — turn by hand' if free else 'torque ON'}).  Ctrl-C to stop.")
        try:
            while True:
                pos, deg = read(bus, wid)
                ld, lm = load(bus, wid)                               # torque proxy (meaningful iff torque on)
                angle = (f"Δ={(((pos - p0 + RES // 2) % RES) - RES // 2) * 360.0 / RES:+7.1f}°"
                         if zero else f"angle={deg:6.1f}°")
                print(f"\r  raw={pos:4d}/4095  {angle}  load={ld}{lm:4d}/1000   ", end="", flush=True)
                time.sleep(1.0 / hz)
        except KeyboardInterrupt:
            print()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--id", type=int, default=6, help="servo id to read (the sensor)")
    ap.add_argument("--hz", type=float, default=20.0, help="poll rate")
    ap.add_argument("--zero", action="store_true", help="report angle relative to the start pose")
    ap.add_argument("--scan", action="store_true", help="just list ids on the bus")
    ap.add_argument("--hold-torque", action="store_true", help="keep torque ON (don't free the shaft)")
    args = ap.parse_args()
    try:
        if args.scan:
            with FeetechBus(args.port) as bus:
                print("ids on bus:", bus.scan(range(0, 40)))
        else:
            run(args.port, args.id, args.hz, args.zero, free=not args.hold_torque)
    except FeetechError as e:
        print("bus error:", e)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
