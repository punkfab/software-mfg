"""feetech_bus.py — drive real STS3215 bus servos over a serial port, reusing the sim codec.

The virtual bus in sim/feetech_protocol.py IS the protocol reference; this points the SAME codec
(build / parse / le / from_le + the STS control table) at a real half-duplex TTL serial port — a
Feetech / Waveshare USB bus-servo adapter. So the packets validated in sim are byte-for-byte what
goes on the wire; there's no second protocol implementation to drift.

    with FeetechBus("/dev/ttyUSB0") as bus:     # 1 Mbps, 8N1, half-duplex
        print(bus.scan())                       # -> ids present
        bus.set_wheel_mode(16)
        bus.sync_velocity({16: 2000, 17: -2000, 18: 0})   # counts/s, sign-magnitude
        bus.stop_all([16, 17, 18])

pyserial is imported lazily, so importing this module (and the kinematics) never needs hardware.
"""
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
import feetech_protocol as fp                    # the SAME codec the virtual bus uses  # noqa: E402

BAUD = 1_000_000
MODE_WHEEL, MODE_POSITION = fp.MODE_WHEEL, fp.MODE_POSITION


class FeetechError(RuntimeError):
    pass


class FeetechBus:
    def __init__(self, port, baud=BAUD, timeout=0.02):
        try:
            import serial
        except ImportError as e:
            raise FeetechError("pyserial not installed — `pip install pyserial` "
                               "(or run with --dry-run to test the kinematics with no hardware).") from e
        self.port = port
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout, write_timeout=timeout)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()

    # --- wire level -------------------------------------------------------------------------
    def _read_status(self):
        """Read one status packet [FF FF id len err params.. chk] -> (id, err, params); None on timeout."""
        deadline = time.time() + 0.06
        hdr = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            hdr = (hdr + b)[-2:]
            if hdr == b"\xff\xff":
                break
        else:
            return None
        meta = self.ser.read(2)                  # id, length
        if len(meta) < 2:
            return None
        length = meta[1]
        rest = self.ser.read(length)             # err + params + checksum
        if len(rest) < length:
            return None
        return fp.parse(b"\xff\xff" + bytes(meta) + rest)

    def _txn(self, id_, instr, params=(), expect_status=True):
        self.ser.reset_input_buffer()
        self.ser.write(fp.build(id_, instr, params))
        self.ser.flush()
        if not expect_status or id_ == fp.BROADCAST_ID:
            return None
        st = self._read_status()
        if st is None:
            raise FeetechError(f"no response from id {id_} (instr {instr:#x})")
        return st

    # --- primitives -------------------------------------------------------------------------
    def ping(self, id_):
        try:
            return self._txn(id_, fp.PING) is not None
        except FeetechError:
            return False

    def read(self, id_, name, signmag=False):
        addr, n = fp.CT[name]
        st = self._txn(id_, fp.READ, [addr, n])
        return fp.from_le(st[2], signmag)

    def write(self, id_, name, value, expect_status=True):
        addr, n = fp.CT[name]
        self._txn(id_, fp.WRITE, [addr, *fp.le(value, n)], expect_status=expect_status)

    def sync_write(self, name, values):
        """One SYNC_WRITE to many servos: params = [addr, n, id, bytes..., id, bytes..., ...]."""
        addr, n = fp.CT[name]
        params = [addr, n]
        for sid, v in values.items():
            params += [sid, *fp.le(v, n)]
        self._txn(fp.BROADCAST_ID, fp.SYNC_WRITE, params, expect_status=False)

    def scan(self, ids=range(0, 32)):
        return [i for i in ids if self.ping(i)]

    # --- EEPROM writes: unlock (Lock=0) -> write -> relock (Lock=1) --------------------------
    def set_id(self, old_id, new_id):
        """Change a servo's ID. Handled specially: after the ID write the node answers on the NEW id,
        and some firmware doesn't ACK the change, so we relock + verify on new_id."""
        if old_id == new_id:
            return
        self.write(old_id, "Lock", 0)
        try:
            self.write(old_id, "ID", new_id)
        except FeetechError:
            pass                                 # firmware may not ACK its own ID change
        time.sleep(0.03)
        self.write(new_id, "Lock", 1)
        if not self.ping(new_id):
            raise FeetechError(f"set_id: no servo answered as {new_id} after the change")

    def set_wheel_mode(self, id_, wheel=True):
        self.write(id_, "Lock", 0)
        self.write(id_, "Operating_Mode", MODE_WHEEL if wheel else MODE_POSITION)
        self.write(id_, "Lock", 1)

    def torque(self, id_, on=True):
        self.write(id_, "Torque_Enable", 1 if on else 0)

    def set_velocity(self, id_, raw):
        self.write(id_, "Goal_Velocity", int(raw))

    def sync_velocity(self, raws):
        self.sync_write("Goal_Velocity", {i: int(v) for i, v in raws.items()})

    def present_velocity(self, id_):
        return self.read(id_, "Present_Velocity", signmag=True)

    def stop_all(self, ids):
        self.sync_velocity({i: 0 for i in ids})


class DryBus:
    """A no-hardware stand-in with the same surface as FeetechBus, for --dry-run: it just logs the
    last velocity command so the kinematics + command path can be exercised with no serial port."""
    def __init__(self, *a, **k):
        self.last = {}

    def close(self): pass
    def __enter__(self): return self
    def __exit__(self, *a): pass
    def ping(self, id_): return True
    def scan(self, ids=range(0, 32)): return []
    def set_id(self, old_id, new_id): print(f"[dry] set_id {old_id} -> {new_id}")
    def set_wheel_mode(self, id_, wheel=True): print(f"[dry] set_wheel_mode {id_} = {wheel}")
    def torque(self, id_, on=True): pass
    def set_velocity(self, id_, raw): self.last[id_] = int(raw)

    def sync_velocity(self, raws):
        self.last.update({i: int(v) for i, v in raws.items()})
        print("[dry] sync_velocity " + "  ".join(f"{i}:{int(v):+5d}" for i, v in raws.items()))

    def stop_all(self, ids):
        self.sync_velocity({i: 0 for i in ids})
