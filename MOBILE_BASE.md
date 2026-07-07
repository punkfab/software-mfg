# Mobile base — the material-handling platform

The near-term milestone: **an SO-101-class arm on a mobile base drives to a floor-standing
3D printer, locks, and pulls a printed part out.** This doc is the design + build plan; the
physics lives in `sim/mobile_base.py` (analytical, gated by `scripts/mobile_base_check.py`),
the deck is `parts/base_deck.py`, and the floor-level planner is `sim/station.py`.

Every number here is a **PREDICTION** until a bench pull-test on a real base anchors the
uncertain params (floor friction `μ`, stepper holding derate, gear efficiency) — that
physical test is exactly the point of building one soon (the calibration layer's reality leg).

## The one thing to internalise: locking wheels does not stop tip-over

An arm reaching out is a force at the tool at some height — a push **and** a tip-over moment
on the base. A "locked" base can give three ways; the real limit is the **worst** of them:

| Mode | Resisted by | Design lever |
|---|---|---|
| **Back-drive** (wheels roll) | motor holding torque ÷ wheel radius | gear it down; brake; feet |
| **Slide** (base skids) | friction `μ·m·g` | mass, tire compound |
| **Tip-over** (rotates about the front edge) | **footprint + CG only** | wide stance, **low CG** |

In the SO-101 printer-pick scenario, **tip-over binds every design** — the motors are never
the limit. Locking the wheels feels like the answer and isn't; **footprint and CG height are.**
Two corollaries the model enforces:

1. **Gear the motors so they aren't the weak link.** Direct-drive NEMA-17 steppers back-drive
   at ~10 N/wheel; a 5:1 planetary moves that above the slide limit, so friction/tip bind first.
2. **Passive mecanum is a trap for an arm platform.** Its transverse rollers free-spin, so a
   locked base resists a *sideways* arm push with almost nothing (~5 N). Use solid wheels,
   **actively-driven** omni rollers, or deployable feet.

## The four designs (SO-101 pulls a 0.25 kg part @ 32 cm reach, 30 cm high; target 15 N hold)

| Design | Fwd hold | Lat hold | Margin | Verdict | Why |
|---|---|---|---|---|---|
| **diff2** — 2wd + front caster | 28.8 N (tip) | 43 N | 130 mm | ✅ simplest | cheapest; watch CG-height lateral tip in turns |
| **mecanum4** — passive rollers | 40.1 N (tip) | **4.8 N** | 126 mm | ❌ | holonomic to drive, **can't hold sideways** |
| **omni_diff** — active rollers | 52.3 N (tip) | 68 N | 149 mm | ✅ | *the wheel in the video* — holonomic **and** locks laterally; gear backlash |
| **footed4** — 4wd + deployable feet | 66.4 N (tip) | 68 N | 189 mm | ✅ robust | drive on wheels, **lock rigid on feet** — best hold, most parts |

**Recommendation for the first build:** start with **diff2** for the fastest path to the
printer-pick test — but *widen the track and keep the arm riser as low as the printer plate
allows* (the diff2 CG-height warning is real), and add a simple **drop-down foot or a servo
brake** for the locked pick so the hold doesn't rely on soft stepper detent. That foot is the
cheap 80/20 of `footed4`. Graduate to **omni_diff** when you want to dock from any angle
(strafe parallel to the printer) with genuine lateral hold — your differentially-driven wheel
is the right mechanism for that, and it's the only holonomic option that actually locks.

## Buyable kit: the 244×198 mm mecanum chassis (ThanksBuyer / AliExpress)

A cheap 4WD mecanum kit (244×198 mm deck, ~80 mm mecanum, DC encoder gearmotors). Run it
through the model on the printer-pick and it **FAILS three ways** — a useful catch before buying:

| | Kit (244×198) | why |
|---|---|---|
| Fwd hold | **6.3 N** (back-drive) | DC gearmotors barely hold; they back-drive before sliding |
| Lat hold | **2.3 N** | passive mecanum rollers free-spin |
| Reach | **tips** at ~0.13 m CG | front support only 122 mm out; the arm's reach CG sits right at the edge |

Dynamically it flips over at the working reach. The binding problems are the **small
footprint** (front edge 122 mm vs a ~0.3 m reach) and **passive rollers + backdrivable
motors**. So: **buy it as a navigation / docking test mule** — it's perfect for proving the
base can drive to the printer and dock (`station.py`) and for building the odometry /
relocalisation stack — but it needs three cheap mods before it can *pick*:

1. **Forward outrigger** — bolt a deck extension + caster or foot ~0.25 m forward so the
   support edge moves out past the arm's reach CG. The single biggest fix (tip-limited → safe).
2. **Drop-down foot** at the front for the locked pick — takes load off the backdrivable
   motors and extends support (recovers the ~6 N back-drive and part of the lateral hold).
3. **Mount the arm low and reaching over the wheelbase**; counterweight the rear. Keep the
   riser as short as the printer plate allows.

With the outrigger + foot it becomes a footed4-lite. Without them it's a drive-only mule.

## Motor assembly spec (diff2 v0)

- **Drive motors:** 2× NEMA 17 bipolar, ~0.59 N·m holding (e.g. StepperOnline `17HS19-2004S1`).
  The arm is light — drive needs only ~0.05 N·m; the motor is sized for **holding**, not go.
- **Reduction:** 5:1 planetary gearbox per motor (StepperOnline `PG` series) or an HTD-3M belt
  reduction. Raises wheel holding to ~2.3 N·m → ~45 N/wheel back-drive resistance (above slide).
- **Rigid-lock option (recommended):** a non-backdrivable **worm gearmotor**, a powered-off
  **brake**, or a **drop foot** — any removes reliance on energized stepper detent for the pick.
- **Closed-loop upgrade:** MKS `SERVO42/57` or StepperOnline `iHSS` closed-loop steppers hold
  position actively (won't silently slip poles under overload) — worth it once loads grow.
- **Wheels:** 100 mm — solid rubber/PU drive wheels for diff2; a 100 mm mecanum/omni **set**
  for the holonomic builds. Heavy-duty swivel **caster** (≈2 in) front, rated ≥ base mass.
- **Drivers:** TMC2209/TMC5160 (SilentStepStick) on a 3D-printer-class board (BTT SKR) or
  external DM542 drivers; closed-loop drivers for the iHSS/SERVO option.
- **Control + odometry:** ESP32/Teensy or a Pi; wheel encoders + a 9-DoF IMU (BNO085) for base
  pose; an AprilTag camera for relocalisation (feeds `Floor.fix` — the base-pose re-anchor).
- **Power:** 12 V (or 24 V for the reduction headroom) LiFePO₄/Li-ion pack + a fused distro.
- **Frame:** the `parts/base_deck.py` plate (300×240×6, corner/arm/motor/caster patterns +
  100 mm wheel cutouts to keep the deck low) on 2020 extrusion or printed standoffs.

## Sources

Steppers/gearboxes/drivers/closed-loop: **StepperOnline**. Wheels (mecanum/omni/solid) +
caster: **Nexus Robot**, **AndyMark**, **RobotShop**, **Pololu**; caster also **McMaster-Carr**.
Extrusion/hardware: **Misumi**, **McMaster**. IMU/encoders/MCU: **Adafruit**, **SparkFun**,
**Pololu**. (Exact part numbers to be pinned per vendor stock at order time.)

## Build & verify plan

1. Print `base_deck.stl`; bolt on 2× NEMA-17 + 5:1 + solid wheels, front caster, battery, MCU.
2. Mount the SO-101 on a **short** riser (keep CG low); wire encoders + IMU.
3. **Bench pull-test** — this anchors the model: pull horizontally at the tool through a force
   gauge until the base moves; record the force **and** the mode (roll / slide / tip). Write it
   back as calibration measurements (`base_mu_slide`, `stepper_hold_derate`) — the numbers above
   flip from PREDICTION to measured.
4. Drive to the printer via `station.py` (collision-checked route + `stage_into_envelope`),
   lock (foot/brake), run the extraction reach, verify the part is grasped and clears the chamber.
5. Compare predicted vs. measured hold + tip margin; log the sim-to-real gap.
