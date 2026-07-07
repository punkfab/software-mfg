"""feetech_protocol.py — make a DC motor speak the Feetech STS/SMS bus protocol.

The SO-101 servos (STS3215) talk a Dynamixel-1.0-style packet protocol over a half-duplex
1 Mbps TTL bus. To put your OWN DC-motor node on that bus — so LeRobot's FeetechMotorsBus
drives it with zero host changes — your firmware must (1) speak the packet protocol and
(2) expose the SAME control-table addresses at the SAME byte widths (little-endian, STS
series). This module is that firmware, written in Python: an exact codec + a virtual
DC-motor servo + a bus, so you can validate the behaviour before touching hardware and use
it as the reference for the real MCU port.

Control-table addresses below are taken verbatim from lerobot's STS_SMS series table, so a
`model="sts3215"` host reads/writes land on the right registers. Packet:
  [0xFF 0xFF | ID | LEN | INSTR | params… | CHECKSUM],  CHECKSUM = ~(ID+LEN+INSTR+Σparams)&0xFF
"""

from __future__ import annotations

from dataclasses import dataclass, field

# --- control table (name -> (address, n_bytes)), from lerobot feetech STS_SMS series ---
CT = {
    "ID": (5, 1), "Baud_Rate": (6, 1), "Phase": (18, 1), "Operating_Mode": (33, 1),
    "Torque_Enable": (40, 1), "Acceleration": (41, 1), "Goal_Position": (42, 2),
    "Goal_Time": (44, 2), "Goal_Velocity": (46, 2), "Lock": (55, 1),
    "Present_Position": (56, 2), "Present_Velocity": (58, 2), "Present_Load": (60, 2),
    "Present_Voltage": (62, 1), "Present_Temperature": (63, 1), "Maximum_Acceleration": (85, 1),
}
RESOLUTION = 4096          # counts/rev (STS3215, 12-bit) — Goal/Present_Position span 0..4095
BROADCAST_ID = 0xFE

# instructions
PING, READ, WRITE, REG_WRITE, ACTION, RESET, SYNC_WRITE = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x83
MODE_POSITION, MODE_WHEEL = 0, 1


def checksum(body: list[int]) -> int:
    return (~sum(body)) & 0xFF


def build(id_: int, instr: int, params=()) -> bytes:
    """Build an instruction (or status) packet. `instr` is the instruction; for a status
    reply it's the error byte — the wire format is identical."""
    params = list(params)
    length = len(params) + 2
    body = [id_, length, instr, *params]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def parse(pkt: bytes):
    """Parse one packet -> (id, instr_or_error, params). Raises on bad header/checksum."""
    if len(pkt) < 6 or pkt[0] != 0xFF or pkt[1] != 0xFF:
        raise ValueError("bad header")
    id_, length, instr = pkt[2], pkt[3], pkt[4]
    n = length - 2
    params = list(pkt[5:5 + n])
    if 5 + n >= len(pkt):
        raise ValueError("truncated")
    if pkt[5 + n] != checksum([id_, length, instr, *params]):
        raise ValueError("bad checksum")
    return id_, instr, params


def le(value: int, n: int) -> list[int]:
    """Encode an int as n little-endian bytes. Velocity is sign-magnitude (bit 15 = sign)."""
    if value < 0:
        value = (abs(value) & 0x7FFF) | 0x8000
    return [(value >> (8 * i)) & 0xFF for i in range(n)]


def from_le(data, signmag=False) -> int:
    v = sum(b << (8 * i) for i, b in enumerate(data))
    if signmag and len(data) == 2 and (v & 0x8000):
        v = -(v & 0x7FFF)
    return v


@dataclass
class VirtualServo:
    """A DC motor pretending to be an STS3215: a 256-byte register file + a closed loop.
    In position mode it seeks Goal_Position; in wheel mode it integrates Goal_Velocity."""
    id: int
    mem: bytearray = field(default_factory=lambda: bytearray(256))
    max_step_per_s: float = 3000.0   # encoder counts/s the DC motor + loop can actually track

    def __post_init__(self):
        self.set("ID", self.id)
        self.set("Torque_Enable", 1)
        self.set("Present_Voltage", 120)     # 12.0 V
        self.set("Present_Temperature", 30)

    def set(self, name, value):
        addr, n = CT[name]
        self.mem[addr:addr + n] = bytes(le(value, n))

    def get(self, name, signmag=False):
        addr, n = CT[name]
        return from_le(self.mem[addr:addr + n], signmag)

    def step(self, dt):
        if not self.get("Torque_Enable"):
            self.set("Present_Velocity", 0)
            return
        pos = self.get("Present_Position")
        if self.get("Operating_Mode") == MODE_WHEEL:
            vel = self.get("Goal_Velocity", signmag=True)
            pos = int(pos + vel * dt) % RESOLUTION
            self.set("Present_Velocity", vel)
        else:
            goal = self.get("Goal_Position")
            err = goal - pos
            step = max(-self.max_step_per_s * dt, min(self.max_step_per_s * dt, err))
            pos = int(pos + step)
            self.set("Present_Velocity", int(step / dt) if dt else 0)
        self.set("Present_Position", pos)

    def handle(self, instr, params):
        """Process an instruction addressed to this servo. Returns a status packet or None
        (writes/sync/broadcast get no reply, matching the real bus)."""
        if instr == PING:
            return build(self.id, 0)                        # status, error=0
        if instr == READ:
            addr, n = params[0], params[1]
            return build(self.id, 0, list(self.mem[addr:addr + n]))
        if instr == WRITE:
            addr, data = params[0], params[1:]
            self.mem[addr:addr + len(data)] = bytes(data)
            return None
        return None


class VirtualBus:
    """The multi-drop bus: routes a packet to the addressed servo, or fans a SYNC_WRITE out
    to many. `process` returns the status bytes a host would read back (or b'')."""

    def __init__(self, servos=()):
        self.servos = {s.id: s for s in servos}

    def add(self, servo):
        self.servos[servo.id] = servo
        return self

    def step(self, dt):
        for s in self.servos.values():
            s.step(dt)

    def process(self, pkt: bytes) -> bytes:
        id_, instr, params = parse(pkt)
        if instr == SYNC_WRITE:
            addr, dlen = params[0], params[1]
            i = 2
            while i + 1 + dlen <= len(params):
                sid, data = params[i], params[i + 1:i + 1 + dlen]
                if sid in self.servos:
                    self.servos[sid].mem[addr:addr + dlen] = bytes(data)
                i += 1 + dlen
            return b""
        if id_ == BROADCAST_ID:
            return b""
        s = self.servos.get(id_)
        if s is None:
            return b""                                       # nobody home -> timeout
        r = s.handle(instr, params)
        return r or b""


class Host:
    """Host-side convenience mirroring FeetechMotorsBus calls, over a VirtualBus (or, later,
    a real serial port with the same build()/parse())."""

    def __init__(self, bus: VirtualBus):
        self.bus = bus

    def ping(self, id_):
        r = self.bus.process(build(id_, PING))
        return bool(r) and parse(r)[0] == id_

    def write(self, id_, name, value):
        addr, n = CT[name]
        self.bus.process(build(id_, WRITE, [addr, *le(value, n)]))

    def read(self, id_, name, signmag=False):
        addr, n = CT[name]
        r = self.bus.process(build(id_, READ, [addr, n]))
        return from_le(parse(r)[2], signmag)

    def sync_write(self, name, id_to_value):
        addr, n = CT[name]
        params = [addr, n]
        for sid, val in id_to_value.items():
            params += [sid, *le(val, n)]
        self.bus.process(build(BROADCAST_ID, SYNC_WRITE, params))


if __name__ == "__main__":
    bus = VirtualBus([VirtualServo(7), VirtualServo(8)])
    host = Host(bus)
    print("ping 7:", host.ping(7), "| ping 9 (absent):", host.ping(9))
    host.write(7, "Goal_Position", 3000)
    for _ in range(200):
        bus.step(0.01)
    print("servo 7 sought Goal 3000 ->", host.read(7, "Present_Position"))
    host.write(8, "Operating_Mode", MODE_WHEEL)
    host.write(8, "Goal_Velocity", 500)
    p0 = host.read(8, "Present_Position")
    for _ in range(100):
        bus.step(0.01)
    print("servo 8 wheel mode: moved", host.read(8, "Present_Position") - p0, "counts in 1s")
    host.sync_write("Goal_Position", {7: 1024, 8: 2048})
    print("sync_write set goals ->", bus.servos[7].get("Goal_Position"), bus.servos[8].get("Goal_Position"))
