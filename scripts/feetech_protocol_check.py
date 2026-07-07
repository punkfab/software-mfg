#!/usr/bin/env python3
"""Gate: the DIY DC-motor servo speaks the Feetech STS protocol well enough to be a drop-in.

Asserts the wire protocol (packet round-trip, checksum, little-endian incl. signed velocity),
the register-map FIDELITY vs lerobot's STS3215 table (the drop-in claim), and the closed-loop
behaviour (position seek, torque gating, wheel-mode integration, sync-write to many).
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import feetech_protocol as fp  # noqa: E402


def main() -> int:
    problems = []

    # 1. packet round-trip + checksum rejection
    pkt = fp.build(7, fp.WRITE, [42, 0xB8, 0x0B])
    if fp.parse(pkt) != (7, fp.WRITE, [42, 0xB8, 0x0B]):
        problems.append("build/parse round-trip mismatch")
    bad = bytearray(pkt); bad[-1] ^= 0xFF
    try:
        fp.parse(bytes(bad)); problems.append("corrupt checksum accepted")
    except ValueError:
        pass

    # 2. little-endian, and sign-magnitude velocity (the STS quirk)
    if fp.le(3000, 2) != [0xB8, 0x0B] or fp.from_le([0xB8, 0x0B]) != 3000:
        problems.append("2-byte little-endian wrong")
    if fp.from_le(fp.le(-500, 2), signmag=True) != -500:
        problems.append("signed (sign-magnitude) velocity round-trip wrong")

    # 3. register-map fidelity vs lerobot STS3215 (the whole point of a drop-in)
    for name, addr in (("Goal_Position", 42), ("Present_Position", 56),
                       ("Operating_Mode", 33), ("Torque_Enable", 40), ("ID", 5)):
        if fp.CT[name][0] != addr:
            problems.append(f"{name} address {fp.CT[name][0]} != STS3215 {addr}")

    bus = fp.VirtualBus([fp.VirtualServo(7), fp.VirtualServo(8), fp.VirtualServo(9)])
    host = fp.Host(bus)

    # 4. discovery
    if not host.ping(7) or host.ping(11):
        problems.append("ping present/absent wrong")

    # 5. position mode seeks the goal
    host.write(7, "Goal_Position", 3000)
    for _ in range(300):
        bus.step(0.01)
    if abs(host.read(7, "Present_Position") - 3000) > 2:
        problems.append(f"position mode did not converge: {host.read(7,'Present_Position')}")

    # 6. torque disable freezes the motor
    host.write(9, "Torque_Enable", 0)
    host.write(9, "Goal_Position", 2000)
    for _ in range(100):
        bus.step(0.01)
    if host.read(9, "Present_Position") != 0:
        problems.append("motor moved with torque disabled")

    # 7. wheel mode integrates velocity, both directions
    host.write(8, "Operating_Mode", fp.MODE_WHEEL)
    host.write(8, "Goal_Velocity", 400)
    p0 = host.read(8, "Present_Position")
    for _ in range(100):
        bus.step(0.01)
    fwd = host.read(8, "Present_Position") - p0
    host.write(8, "Goal_Velocity", -400)
    p1 = host.read(8, "Present_Position")
    for _ in range(50):
        bus.step(0.01)
    rev = host.read(8, "Present_Position") - p1
    if not (390 <= fwd <= 410):
        problems.append(f"wheel mode fwd integration off: {fwd} (expect ~400)")
    if rev >= 0:
        problems.append(f"negative velocity did not reverse: {rev}")

    # 8. sync-write commands many in one packet
    host.sync_write("Goal_Position", {7: 1024, 9: 512})
    if bus.servos[7].get("Goal_Position") != 1024 or bus.servos[9].get("Goal_Position") != 512:
        problems.append("sync_write did not set all goals")

    print("wire: packet round-trips, bad checksum rejected, LE + signed velocity ok")
    print(f"map:  Goal_Position@{fp.CT['Goal_Position'][0]} Present_Position@{fp.CT['Present_Position'][0]} "
          f"Operating_Mode@{fp.CT['Operating_Mode'][0]} == lerobot STS3215 (drop-in)")
    print(f"loop: pos seek -> {host.read(7,'Present_Position')} (goal was 3000 pre-syncwrite), "
          f"wheel +{fwd}/{rev} counts, torque-off frozen, sync_write to 2 ids")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: a DC motor running this speaks STS3215 well enough for LeRobot to drive it unchanged")
    return 0


if __name__ == "__main__":
    sys.exit(main())
