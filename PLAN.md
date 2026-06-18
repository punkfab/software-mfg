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

- [ ] Initialize git repo; commit `CONCEPT.md` + `PLAN.md`.
- [ ] Project layout:
  ```
  parts/        # build123d part scripts (e.g. bend_disc.py, shear mounts)
  assemblies/   # partcad assembly definitions (assemblies-as-code)
  sim/          # MuJoCo MJCF models + scenes
  orchestration/# operation-graph + scheduler (later phases)
  exports/      # generated STEP / 3MF / STL (gitignored or LFS)
  ```
- [ ] Pin toolchain: Python env, `build123d`, `partcad`, `mujoco`. Record versions.
- [ ] Migrate existing `bend_disc.py` variant family into `parts/`; declare it as
      a partcad package (first dependency-graph node).
- [ ] CI/check: a script that regenerates STEP/STL from each part script
      (catches geometry regressions; watertight check).

## Phase 1 — SO-101 in simulation

Goal: a controllable SO-101 in MuJoCo.

- [ ] Obtain/verify SO-101 MJCF (LeRobot ecosystem); import into `sim/`.
- [ ] Stand up a basic scene (arm + table + a fiducial/datum).
- [ ] Validate forward/inverse kinematics; scripted joint moves.
- [ ] Decide control interface (direct joint targets vs. IK helper) for later
      orchestration to drive.

## Phase 2 — Tool changer + self-actuated shear (sim)

Goal: model the modular end-effector concept and one process tool.

- [ ] Model the **kinematic coupling** interface (tool-side + arm-side) in build123d.
- [ ] Model a **self-actuated shear** tool (its own actuator DOF in MJCF).
- [ ] Model an **EPM/retention** abstraction in sim (attach/detach event, not full
      magnetics) and a **tool rack** with parked tools.
- [ ] Sim sequence: arm approaches rack → couples shear → lifts → (carries) →
      returns/decouples. Measure seating repeatability proxy.

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
