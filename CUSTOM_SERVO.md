# DIY serial-bus servo — make a DC motor speak the Feetech STS protocol

Goal: put a plain DC gearmotor on the SO-101's serial bus so **LeRobot drives it with zero
host changes** — it just looks like another `sts3215` ID. The reference implementation is
`sim/feetech_protocol.py` (codec + a virtual DC-motor servo + bus), gated by
`scripts/feetech_protocol_check.py`. Port that behaviour to an MCU and you have real hardware.

## What "speak the protocol" actually requires

Two things, both proven in the sim:

1. **The wire protocol** — half-duplex TTL UART, 1 Mbps, packets
   `[0xFF 0xFF | ID | LEN | INSTR | params… | CHECKSUM]`, checksum `~(ID+LEN+INSTR+Σparams)&0xFF`.
   Handle at least `PING`, `READ`, `WRITE`, `SYNC_WRITE`. Reply only when addressed by your ID
   (or on a READ/PING); stay off the line otherwise.
2. **Register-map fidelity** — expose the SAME control-table addresses/widths LeRobot's
   `model="sts3215"` uses, **little-endian** (STS series): `ID@5, Baud@6, Operating_Mode@33,
   Torque_Enable@40, Goal_Position@42 (2B), Goal_Velocity@46 (2B, sign-magnitude),
   Present_Position@56 (2B), Present_Velocity@58, Present_Voltage@62, Present_Temperature@63`.
   Position spans 0..4095 (12-bit, 4096/rev). Match these and the host can't tell you apart.

## Hardware

| Block | Part | Note |
|---|---|---|
| MCU | RP2040 / STM32 / ESP32 (one per motor, or one per few) | needs a spare UART; RP2040 PIO does clean 1 Mbps half-duplex |
| Half-duplex bus | tie TX+RX to the single data line via a **tri-state buffer** (74LVC1G125) with DIR from a GPIO, or the Feetech-style diode+pull circuit | drive TX only during your reply, then release; a **return delay** avoids collisions |
| Motor driver | DRV8833 / TB6612 (≤~1–2 A) or BTS7960 (big) | H-bridge, PWM in |
| Feedback | quadrature encoder, or a magnetic **AS5600** (absolute, I²C) | AS5600 gives absolute 0..4095 directly — maps 1:1 to Present_Position |
| Power | share the bus V+ (12 V) via a local tap; add bulk cap | don't pull motor current through the thin daisy-chain — see MOBILE_BASE.md |

## Firmware = the sim, ported

The control loop is exactly `VirtualServo.step()`:
- **Position mode** (`Operating_Mode=0`): PID the encoder to `Goal_Position`, write back `Present_Position/Velocity`.
- **Wheel mode** (`Operating_Mode=1`): PWM proportional to `Goal_Velocity` (sign-magnitude), integrate `Present_Position` (wraps).
- **`Torque_Enable=0`** → coast (H-bridge off). Honour `ID`/`Baud` writes so `set_motor_id.py` works.

Bring-up order: (1) echo/ping on the bus; (2) implement READ/WRITE against the register file;
(3) close the motor loop; (4) point a **real `FeetechMotorsBus` at it** and confirm it enumerates
and moves alongside the arm. Test each step against `feetech_protocol.py` on a PTY first.

## Cheaper servos — buy-vs-DIY, and the protocol trap

The cheapest way to *more* of these is not always a clone:

- **Same protocol (drop-in):** **Waveshare** ST/SC-series (e.g. `ST3215`) are rebadged Feetech —
  same silicon, same protocol, often cheaper. Buying **Feetech direct** (Alibaba) in quantity is
  usually the floor price. These need no firmware — they already are STS3215s.
- **NOT the same protocol (trap):** the *generic* cheap bus servos — **HiWonder / LX-16A /
  LewanSoul / Lobot** — speak a different protocol (`0x55 0x55` header, own checksum), so they
  will **not** drop into LeRobot's Feetech bus. Cheap, but a different driver + wiring.
- **DIY = the ultimate cheap copy:** a bare DC gearmotor + AS5600 + a $1 MCU running the firmware
  above is *the* low-cost node, and you own the register map — you can add registers the STS
  doesn't have (current sense, foot-deploy, temperature) while staying a drop-in for the standard
  ones. This is the right path when you want many cheap wheel/aux actuators that the arm host
  already knows how to talk to.

Rule of thumb: buy **Waveshare/Feetech** for a few extra *matched* joints; **DIY** for a fleet of
cheap wheel/aux motors on the same bus. Avoid LewanSoul-protocol servos unless you'll run a
separate driver for them.
