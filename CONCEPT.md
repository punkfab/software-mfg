# Software-Defined Manufacturing & Assembly — Concept

## 1. Vision

Treat physical fabrication and assembly the way we treat software: as something
an agent can **define declaratively, iterate on, simulate, verify, and optimize
automatically**. The goal is a workcell where small robot arms and process
machines (a CNC wire bender, a 3D printer, shears, drivers, insert presses)
cooperate to build multi-step, multi-process assemblies — and where the *design
and the process improve themselves* as the design solidifies.

The north star: **rapid physical prototyping that loops like Claude Code** —
propose a design/assembly, evaluate it, refine it — but grounded in the reality
that atoms are slow and semi-irreversible.

## 2. The core insight: two loops, not one

Software iteration is ~free and reversible. Physical iteration costs hours,
material, and is hard to undo. So the architecture is **two coupled loops**:

- **Fast inner loop — simulation (MuJoCo).** ~95% of iterations happen here.
  Generate a candidate (geometry, motion, assembly sequence), evaluate, keep or
  discard. Effectively free. This is where the "agentic" iteration lives.
- **Slow outer loop — physical verification.** Only sim-winners get printed and
  assembled. The physical world's job is to *correct the simulator's lies*
  (close the sim-to-real gap) and confirm function — not to be the iteration
  substrate.

**Implication:** effort goes into making the simulator faithful enough that
sim-winners are usually real-winners. We do **not** oversell autonomous physical
trial-and-error; the automatable part today is the sim loop + generative DFM,
with sparse, expensive physical confirmation and human-assisted builds.

## 3. Guiding principles

1. **Coarse-by-base, fine-by-local-datum.** Mobile bases repeat to ~cm; never
   trust global pose for precision. Accuracy comes from *local* features —
   fiducials, kinematic couplings, fixtures — not global position.
2. **Separate manipulation from process force.** The arm positions; the tool
   supplies its own process force (cutting, pressing, driving). This keeps the
   arm requirements low and makes tools self-contained.
3. **Design-as-code, all the way down.** Every part, assembly, and process step
   is a versioned, diffable artifact. If it can't be diffed and regenerated, an
   agent can't iterate on it.
4. **Simulate before you cut.** No physical action without a sim model of it.
5. **The operation graph is the program.** Manufacturing = a DAG of operations
   over resources. Optimizing that graph (sequencing, parallelism, part
   consolidation) is the real "software-defined" payload.
6. **Compose cells, don't fork them.** A machine that already exists as its own
   parts-as-code repo (e.g. the wire bender) is consumed *by reference*: the
   orchestrator invokes the cell's own toolchain to emit geometry/behavior. The
   cell stays the single source of truth and is never modified by software-mfg
   (sync leaves no artifacts in the cell tree).

## 4. System architecture — the format/software stack

These formats are **layers, not competitors**:

| Layer | Tool / format | Responsibility | Explicitly NOT |
|---|---|---|---|
| Part definition | **build123d** | Parametric, code-defined geometry | Assembly/process semantics |
| Exact interchange | **STEP (AP242)** | B-rep solids + assembly tree + PMI | A print job |
| Package / compose | **partcad** | Version parts, assemblies-as-code, dependency graph | Physics, slicing |
| Print handoff | **3MF** | Multi-object build-plate layout, transforms, materials, units | Parametric / exact geometry |
| Validate | **MuJoCo (MJCF)** | Robot + part kinematics & dynamics | Manufacturing-grade geometry |
| Orchestrate | **Operation graph + scheduler** (to build) | Precedence, resources, timing, optimization | — |

Pipeline:
**build123d (define) → partcad (version/compose) → STEP (exact) + 3MF (print) →
MuJoCo (validate) → operation graph (orchestrate).**

Format notes:
- **STL is a dead end** here — no units, no assembly, no materials. Slicer input only.
- **3MF ≫ STL.** Zip/XML container with a build/component model (objects placed
  by transform = an assembly layout), materials, units, and an extensions
  mechanism (Production, Slice, Beam Lattice). Right format for "print a small
  sub-assembly on one plate."
- **partcad = "npm for hardware."** Parts declared from build123d/CadQuery/
  OpenSCAD/STEP, composed into assemblies with positions, versioned, importable
  like dependencies. This dependency graph is the substrate an agent mutates
  when it merges parts or swaps a variant. The existing bend-disc variant family
  (swept `PIN_OFFSET` / `MANDREL_D`) is already a partcad-shaped problem. It is
  the *intended* registry layer; standing up its sandboxed runtime is deferred
  (see PLAN.md). Until then, cells are composed by the lightweight bridge below.
- **Cells composed by reference (`cells.yaml` + `scripts/sync_cells.py`).** An
  external machine repo (the wire bender) is declared as a *cell*; sync invokes
  the cell's own interpreter on its own CAD to emit STEP/STL into
  `exports/cells/`, then re-imports the STEP under our build123d as an integrity
  gate. The cell repo is never written to. This is the working stand-in for the
  partcad registry and embodies principle #6.

## 5. Hardware concepts

### 5.1 Manipulator baseline — SO-101
Start with commercial **SO-101** arms instead of a custom design. Rationale:
decouples the manipulator problem from the process/coordination problem; existing
URDF/MJCF in the LeRobot ecosystem gives a head start in MuJoCo. Constraint:
low payload/stiffness (Feetech servos) — it cannot supply process force.

### 5.2 Modular powered end-effector + tool changer
Arm performs its own tool changes, picking process tools off a rack.

- **Repeatability: kinematic coupling preferred over locating pins.** A Maxwell
  coupling (3 balls in 3 vees) seats deterministically to microns, self-centers,
  and won't jam. Pins-in-holes over-constrain and wear. (Ref: Jubilee / E3D
  tool-changers.) Locating pins remain a fallback.
- **Retention: electropermanent magnet (EPM) preferred over plain electromagnet.**
  EPM pulses to toggle, holds with zero standing power, and fails safe. A plain
  electromagnet burns power continuously and drops the tool on power loss.
- **Cross-interface power/comms is half the design.** Tools are *powered*
  (self-actuated). Carry power/signal across the coupling with **pogo pins**,
  which benefit from the kinematic repeatability. Optionally pneumatics.

### 5.3 Process tools (self-actuated)
- **Shear** — solves the wire bender's missing integrated cutter. Mounted as an
  end-effector tool; arm positions, the shear supplies its own cutting force
  (solenoid / motor+leadscrew / pneumatic) because SO-101 joints cannot shear
  spring/music wire.
- Future tools: gripper, screwdriver, **heat-set-insert press**, dispenser.

### 5.4 Mobile base
Gives the arms more DOF and lets them move in the workcell (more to simulate).
Trivial to add in MuJoCo (planar/free joint). Governed by principle #1:
coarse positioning by base, fine positioning by arm relative to a local datum.

## 6. The orchestration core — operation graph

The component list isn't the system; the **operation graph** is. A build is a
DAG of operations (bend → cut → place → heat-set insert → fasten) over resources
(machines, tools, fixtures) with precedence and timing constraints. This is a
build system for atoms — a Makefile whose targets are physical states.

Two automatable optimizations fall out, and both are what we ultimately want:

1. **Part consolidation ("merge parts").** A checkable graph test
   (Boothroyd–Dewhurst DFA relative-motion criterion): two adjacent parts can
   merge iff there is no relative motion, no differing material requirement, and
   no service/disassembly need. 3D printing uniquely exploits this (print
   consolidated geometry you couldn't mold/machine). The agent proposes merges;
   the sim confirms function survives.

2. **Cycle-time optimization.** Clear objective (seconds/unit), evaluator already
   exists (MuJoCo). Levers: batch same-tool ops to eliminate tool changes
   (job-shop sequencing), overlap operations (arm assembles unit N while bender
   forms unit N+1), reduce travel, and — feeding back — every consolidated part
   removes a pick-place-fasten cycle. A legitimate search/RL target with a real
   reward signal.

> "It gets faster automatically" is credible **as scheduling + DFA over a sim**,
> not as autonomous physical trial-and-error.

## 7. Scope boundaries / honesty

- Full autonomous *physical* iteration is **not** claimed. Physical builds are
  human-assisted and sparse; autonomy lives in simulation and design generation.
- Precision is bounded by local datums, not by robot global accuracy.
- The hardest, highest-value software is the **orchestration/scheduler layer**,
  not any single CAD part.

## 8. Glossary

- **DFA / DFM** — Design for Assembly / Manufacturing.
- **EPM** — Electropermanent magnet (toggled by a pulse, zero holding power).
- **Kinematic coupling** — exact-constraint mount (e.g. 3 balls / 3 vees) for
  micron-repeatable seating.
- **MJCF** — MuJoCo's model format.
- **Local datum** — a local geometric reference (fiducial/fixture/coupling) that
  supplies precision independent of global robot pose.
- **Operation graph** — DAG of physical operations over resources; the "program"
  that the scheduler optimizes.
