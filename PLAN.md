# Software-Defined Manufacturing — Plan

Actionable roadmap for the vision in [CONCEPT.md](./CONCEPT.md). Sequenced to cut
one honest vertical slice through every layer *before* scaling breadth.

## Strategy

- Build the **thinnest end-to-end slice first**, then widen. Avoid perfecting any
  single layer (CAD part, scheduler, sim) in isolation.
- Keep the **simulation loop** as the primary iteration engine; treat physical
  builds as sparse verification milestones.
- Everything is design-as-code and versioned from day one.

## Phase 0 — Foundation & repo skeleton

Goal: a working, versioned project that can define a part and open it in sim.

- [x] Initialize git repo; commit `CONCEPT.md` + `PLAN.md`.
- [x] Project layout:
  ```
  parts/        # build123d part scripts authored HERE (each exposes `part`)
  assemblies/   # partcad assemblies-as-code (later phases)
  sim/          # MuJoCo MJCF models + scenes
  orchestration/# operation-graph + scheduler (later phases)
  scripts/      # check_parts.py (local parts) + sync_cells.py (external cells)
  exports/      # generated STEP / 3MF / STL (gitignored); cells/ for cell output
  cells.yaml    # external cells composed by reference (see below)
  ```
- [x] Pin toolchain: Python 3.12.9, build123d 0.10.0, mujoco 3.9.0, partcad 0.7.135
      (+ rich_click). Recorded in `pyproject.toml` / `README.md`.
- [x] CI/check: `scripts/check_parts.py` regenerates STEP/STL from each local part
      and gates on single-watertight-solid + positive volume.

### Compose the wire bender, don't fork it

The wire bender is its own mature repo (`../wirebender`: CAD + MuJoCo sim +
slicer). So `bend_disc` is **not** migrated into `parts/` — that would fork it.
Instead software-mfg treats wirebender as an **external cell composed by
reference**:

- [x] `cells.yaml` declares cell parts (path, the cell's own interpreter, builder fn).
- [x] `scripts/sync_cells.py` invokes the cell's **own** toolchain to emit
      STEP/STL into `exports/cells/<cell>/`, then re-imports the STEP under our
      build123d as an integrity gate. Verified: `wirebender/bend_disc` → valid
      single solid, 8257 mm³.
- [x] **Constraint: never modify a cell repo.** The cell stays the single source
      of truth; sync runs with `PYTHONDONTWRITEBYTECODE=1` so it leaves no
      artifacts (not even `__pycache__`) in the cell tree.

- [ ] **partcad as the registry layer (deferred).** partcad is the intended
      formalization (versioned package graph over the same parts). Blocked: its
      runtime wants an isolated Python 3.11 sandbox with CAD deps pip-installed,
      which isn't present here. Revisit to publish wirebender as a partcad
      package once that runtime is stood up.

## Phase 1 — SO-101 in simulation

Goal: a controllable SO-101 in MuJoCo.

- [x] Obtain/verify SO-101 MJCF; import into `sim/`. **Official MJCF exists** —
      `TheRobotStudio/SO-ARM100` ships `Simulation/SO101/` (Apache-2.0). Vendored
      a pinned snapshot to `sim/so101/` (commit `aec17bb`; see `PROVENANCE.md`).
      6 DOF: shoulder_pan/lift, elbow_flex, wrist_flex/roll, gripper.
- [x] Validate FK + scripted joint moves. `scripts/so101_check.py` compiles the
      scene, runs a position-control move, confirms tracking (0.000 rad err) and
      end-effector motion (0.265 m). Gates the vendored model.
- [x] Decide control interface: **position actuators** (STS3215 class, kp≈998);
      `data.ctrl[:] = desired joint angles (rad)`. Orchestration drives this.
- [x] Workcell scene (arm + table + fiducial/datum), composed from the *outside*
      via the MjSpec API (`sim/workcell.py`) — the vendored snapshot is untouched.
      Defines `DATUM_POS`, the local reference for fine positioning.
- [x] IK helper (`sim/ik.py`, damped least squares over the 5 arm joints).
      `scripts/workcell_check.py` gates it: IK converges (0.4 mm) and the arm
      reaches the datum under position control (0.4 mm).

## Phase 2 — Tool changer + self-actuated shear (sim)

Goal: model the modular end-effector concept and one process tool.

- [x] **Kinematic coupling** in build123d (`parts/coupling_{arm,tool}_side.py`):
      Maxwell 3-vee / 3-ball pair + central EPM/pogo bore. Watertight, validated.
- [x] **Self-actuated shear** tool in MJCF (`sim/toolchanger.py`): free body with
      its own hinged blade + position actuator — the arm only positions it.
- [x] **EPM/retention** abstraction: a weld equality toggled via `data.eq_active`,
      welded *in place* (relpose captured at the couple instant → zero snap). Plus
      a **tool rack** cradle, contact-isolated so the tool only touches its rack.
- [x] Sim sequence + gate (`scripts/toolchange_check.py`, `toolchange_demo.py`):
      approach → couple → lift (+73 mm) → present at datum (9 mm) → shear (0.81 rad)
      → return (4 mm) → decouple. Carry gap tracks the weld offset exactly (50 mm).
- [ ] Seating-repeatability proxy (couple under jittered approach, measure pose
      spread) — deferred; motivates the physical kinematic coupling's self-centering.

## Printer cell — Bambu P1S (eject-in-place)

A second manufacturing cell: the 3D printer, enabling the print→eject→iterate
loop. Decisions: **P1S (enclosed)**, **eject-in-place** removal (cool → open door
→ full-width sweep pushes the part out the front into a bin).

- [x] Sim: `sim/printer_cell.py` (MjSpec) — **kinematically faithful P1S CoreXY**:
      bed on a Z-slide, toolhead on X/Y slides at fixed gantry height (never Z).
      Eject uses the **toolhead to knock** the cooled part off the front edge (no
      added hardware — the standard farm trick) → bin; the bed then drops
      (post-cycle). `eject_demo.py` renders, `eject_check.py` gates it (part on bed
      → knocked past front edge → in bin). PASS. The bed actuator is modeled stiff
      (leadscrew) so it holds the bed against gravity.
- [x] CAD: `parts/ejector_blade.py` — full-width sweep blade, kept as an *optional*
      deployable bolt-on for parts that don't knock cleanly; default is the toolhead.
- [ ] Control adapter (the cell's software side): LAN/MQTT via `bambulabs-api` /
      `pybambu` — start a sliced 3MF print, read bed-temp / progress / done.
      Needs P1S **Developer/LAN mode** enabled (note: Jan-2025 firmware added auth
      that broke third-party control; dev-mode toggle restores it).
- [ ] CAD the **auto door-opener** (servo/linear pusher) + the cooldown step.
- [ ] Future upgrade: flex-plate **swap + magazine** for tall/enclosed parts
      (reuses the tool-changer + kinematic-coupling work for plate registration).
- [ ] Scheduling: overlap cooldown of unit N with printing of unit N+1.

## Phase 3 — First vertical slice (the milestone)

> **SO-101 in MuJoCo, with a self-actuated shear end-effector, performing one
> tool-change and one cut on a simulated wire from the bender — driven by a
> 3-node operation graph (form → present → shear).**

- [ ] Minimal simulated wire (deformable or segmented proxy) emerging from a
      stand-in bender.
- [ ] 3-node operation graph: `form → present → shear`, with precedence + a
      timing handoff between bender and arm.
- [ ] Minimal scheduler executes the graph against the sim.
- [ ] Success criteria: cut occurs at the right wire location within tolerance;
      handoff timing holds; run is repeatable across N sim seeds.

This slice de-risks: MJCF import, coupling/datum model, machine↔arm timing, and
the op-graph scheduler — one cut through every layer.

## Phase 4 — Orchestration layer

Goal: turn the toy graph into a real operation-graph engine.

- [ ] Define the operation-graph schema (nodes = ops, edges = precedence;
      resources = machines/tools/fixtures; per-op duration + tool requirement).
- [ ] Scheduler that assigns ops to resources respecting precedence.
- [ ] **Cycle-time metric** computed from a sim run (the reward signal).
- [ ] First optimizations: batch same-tool ops (eliminate tool changes); overlap
      independent ops (arm works while bender forms next piece).

## Phase 5 — Generative DFM (the self-improvement payload)

Goal: the design/process improves as it solidifies.

- [ ] **Part-consolidation analyzer**: build the assembly adjacency graph; apply
      the DFA relative-motion test (no relative motion + same material + no
      service need → merge candidate). Propose merges; sim confirms function.
- [ ] **Cycle-time search**: agent proposes sequencing/parallelism/consolidation
      changes; MuJoCo evaluates; keep improvements. Log per-unit time trend.
- [ ] Variant management via partcad (sweep parameters; version winners).

## Phase 6 — Physical verification (sparse outer loop)

Goal: close the sim-to-real gap on real hardware.

- [ ] Physical SO-101 + printed kinematic coupling + shear tool.
- [ ] Heat-set-insert workflow (incremental assembly) — human-assisted.
- [ ] Measure real seating repeatability & cut accuracy vs. sim; feed corrections
      back into the simulator.
- [ ] Validate one consolidated design against its multi-part predecessor.

## Phase 7 — Mobile base & multi-arm (scale breadth)

- [ ] Add mobile base (planar/free joint) in MuJoCo; apply coarse-by-base /
      fine-by-datum control.
- [ ] Fiducial-based local relocalization for fine ops after base moves.
- [ ] Multi-arm coordination in the scheduler (parallel resources).

## Cross-cutting / ongoing

- [ ] Keep a sim-to-real gap log (predicted vs. measured) per physical milestone.
- [ ] Every part/assembly stays regenerable from code; no orphan binary CAD.
- [ ] Document decisions in CONCEPT.md as they firm up.

## Open questions / decisions to make

- [ ] SO-101 MJCF: which community model is canonical / best maintained?
- [ ] partcad assembly (`ASSY`) format — exact capabilities & constraints model?
- [ ] Shear actuation: solenoid vs. motor+leadscrew vs. pneumatic (force/size/speed)?
- [ ] EPM sourcing/feasibility vs. starting with a plain electromagnet for v1?
- [ ] Wire model in MuJoCo: deformable cable vs. segmented rigid proxy fidelity?
- [ ] Scheduler approach: hand-rolled heuristic vs. existing job-shop solver?

## Definition of done (program-level)

A workcell where: an assembly is defined as code; the agent iterates it in
simulation; a scheduler sequences a multi-process build; cycle time measurably
drops as the design solidifies (via consolidation + sequencing); and physical
builds confirm the simulated result within a tracked sim-to-real tolerance.
