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

## 1a. Strategic thesis — why this becomes table stakes

The bet behind this project: **vertical integration plus software-defined,
end-to-end simulation of the entire manufacturing process becomes a competitive
necessity, not a luxury.** As product cycles shorten and supply chains stay
volatile, the firms that win will be the ones that can model and re-plan the
*whole* pipeline in software — design, tooling, process, sourcing, assembly — and
change it on the fly instead of through months of manual re-industrialization.

Concretely, the simulator and the operation graph must eventually carry more than
geometry and motion:

- **Tooling is in the loop, not a given.** End-effectors, fixtures, the shear, the
  tool changer are designed, simulated, and costed alongside the product. A new
  variant may imply a new tool — and the system should weigh *designing + printing
  it* against *buying it*.
- **Build-vs-buy is a graph decision.** Every operation/part can be made in-house
  (a cell with a cycle time + capital cost) or sourced (a supplier with a price +
  lead time + risk). The operation graph is the natural home for this: a node's
  resource is either an internal cell or an external supplier, and the
  scheduler/optimizer weighs cycle time, cost, capital, and risk together.
- **Sourcing & materials are parameters, not afterthoughts.** Material
  availability, substitutions, and supplier lead times live in the model, so the
  same engine that minimizes cycle time can re-plan around a shortage or a price move.
- **Just-in-time process modification is the edge.** When the product changes (a
  variant) or the supply changes (a material/supplier swap), the operation graph
  **re-plans automatically** — re-schedule, re-tool, re-source — rather than
  re-industrializing by hand. That agility *is* the competitive advantage.

What's built so far is the substrate for exactly this: parts and tools as code,
cells composed by reference, and an operation graph whose scheduler already turns
cycle time into a measured, optimizable signal. Build/buy, sourcing, lead time,
and cost are additional attributes on those same operations and resources — the
roadmap (Phase 5+) extends the optimizer from "minimize cycle time" to "plan the
whole vertically-integrated process under product and supply variation."

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
- **Operation round-trip / human editing (`featuretree/`).** No neutral *file*
  preserves a feature tree across tools (topological naming + no shared op
  ontology), so we keep a neutral **feature IR (DSL)** and emit it into each
  tool's native tree. FreeCAD is the first backend (its own `freecadcmd` → a real
  `.FCStd` tree); human edits flow back into the IR by feature name. Roadmap:
  face/edge selectors (query-based, to face topological naming), a build123d
  emitter, then Onshape FeatureScript / Fusion API / a SolidWorks macro. This is
  how a human gets back into the loop on a code-defined part.
- **Two repos, two halves of one system.** `../robot-effectors` is the *physical*
  CAD half — the real end effectors (Fidlock magnet-triggered/mechanically-held
  tool changer, 3-vee coupling, dock, pogo-pin power pass-through, rack&pinion
  gripper, SO-101 adapter); `software-mfg` is the *sim + orchestration* half. The
  changer/gripper geometry is canonical there and should be **composed by
  reference** (read-only, its own interpreter — like wirebender), not re-modelled
  here. Our sim's EPM-weld retention is a simplification of their real
  **dock-actuated Fidlock latch** (the dock locks/releases, the arm only docks).
- **Cells composed by reference (`cells.yaml` + `scripts/sync_cells.py`).** An
  external machine repo (the wire bender) is declared as a *cell*; sync invokes
  the cell's own interpreter on its own CAD to emit STEP/STL into
  `exports/cells/`, then re-imports the STEP under our build123d as an integrity
  gate. The cell repo is never written to. This is the working stand-in for the
  partcad registry and embodies principle #6.
- **Each machine is a cell with a physical + a control side.** The Bambu P1S
  printer cell (`sim/printer_cell.py`) models the *physical* side — CoreXY bed (Z)
  + toolhead (XY), door, eject-in-place by toolhead-knock — and gets a *control*
  adapter (LAN/MQTT: start print, read
  bed-temp/done). The wire bender is a cell; the printer is a cell; tools are
  cells. The scheduler sequences operations across them (e.g. overlap a part's
  cooldown with the next print). Control-side caution: Bambu's Jan-2025 firmware
  added LAN auth — third-party control needs Developer/LAN mode enabled.

- **The sim is a cache of reality (`calibration/`).** Sim-CI tests a model but ships
  a physical part, so a green sim only counts where its parameters are anchored to
  reality. The calibrated parameter vector (value + σ + operating point + envelope +
  which build last confirmed it) is the real source of truth; sims read it,
  measurements write it back as a reviewable diff, and every result carries a
  staleness stamp — FRESH / STALE / EXTRAPOLATING — counted in builds-since-anchor.
  This is grey-box system ID, not reverse-engineering: we keep the causal structure
  and fit the parameters.

- **Foil as feedstock (`sim/foil_former.py`, `sim/foil_lom.py`).** Common aluminum
  foil is the ultimate compose-don't-fork stock (<50¢/m², ubiquitous) and a probe of
  the machines-making-machines loop. Two cells: a *foil former* (the wire bender lifted
  a dimension — a (feed, bend) program folds flat stock into geometrically-stiff
  profiles, with calibrated springback + work-hardening) and a *foil LOM printer*
  (slice a solid into ~10µm layers, bond + cut + stack — thin foil means thousands of
  layers, which the planner reports honestly). No physical former exists yet, so their
  parameters are unanchored and every result reads as a PREDICTION — the calibration
  layer doing its job on a brand-new cell.

## 4a. Process design space

The processes above (and the candidates we keep surveying — pulse/spark forming,
vaporizing-foil impact welding, cold spray, plasma/PVD deposition, electrophotographic
powder printing) are mapped and scored in **[PROCESS_SURVEY.md](./PROCESS_SURVEY.md)**.
The load-bearing principle: this architecture **rewards processes whose geometry is set
by a digital pattern or a deterministic path** (bitmap-per-layer, toolpath, bend
program) and **punishes stochastic thermal/plasma parameter soup** (EDM, plasma spray,
PVD), where the output is an empirical function of a dozen coupled knobs. When a process
is soup, consume it as a black box or don't use it — don't pretend to software-define
it. That single test decides build-vs-consume-vs-reject for every process.

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
- **Press** — self-reacting C-frame for seating bearings/inserts (`sim/press_cell.py`).
- **Hot-glue applicator** (`parts/glue_*.py`, `sim/glue_cell.py`) — a switchable
  dispenser on the *same* coupling interface as the shear (so the changer already
  handles it). Build/buy like the shear: printed body + motorized stick feed (build),
  melt cartridge + nozzle held off by a **thermal break** (buy). Gluing is nearly
  force-free — the physics is thermal (melt-ready) + extrusion + open/set time, and
  those set/open times drive **multi-arm coordination** (§6): the first task that
  genuinely needs two arms (one holds the joint while the other glues, and keeps
  holding until it sets).
- Future tools: gripper, screwdriver, dispenser variants.

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

### 6a. From CAD to motion — the end-to-end assembly pipeline

The op-graph is the plan; this is how a plan becomes a *verified* physical build. One
sample assembly runs the whole stack (`orchestration/assemble.py`):

1. **Reference** — the CAD assembly (`tracking/assembly.py`): parts + nominal poses +
   mates in the work datum frame. The STEP/CAD assembly is both the **plan** (where each
   part goes) and the **check** (did it land there).
2. **Plan** — an op-graph over arm/tool/cure resources (§6).
3. **CAM** — a process op becomes a **toolpath** (`orchestration/toolpath.py`): one
   generator per operation — `bead` (dispense along a seam), `insert` (plunge for a
   press / peg-in-hole), `pick_place` (transport with a lift) — each an ordered list of
   tool-tip targets + per-point normals, so motion and interference are uniform across ops.
4. **Clearance** — before any motion runs, the toolpath is a **swept solid-interference
   check** (`sim/interference.py`): the tool (or carried part) mesh is stepped along the
   path and boolean-intersected against the workpiece/fixtures. It answers what a toolpath
   alone can't — *does the moving solid foul anything* — and returns the first colliding
   waypoint so motion stops before it. Self-contained (AABB broad phase → manifold3d
   narrow phase, no fcl); a mesh that can't be closed for the boolean falls back to its
   convex hull and is flagged `approximate` — **conservative, never a silent clear.**
5. **Motion** — the toolpath is solved to joint waypoints by IK. The solver is
   **pluggable**: a built-in positional DLS solver (`sim/ik.py`) keeps everything
   self-contained; **Placo** (QP, orientation-aware) drops in for real following, where
   the tool must stay normal to the surface along the path.
6. **Execute + track** — the world model (`tracking/`) follows every part: which arm
   holds it, its estimated pose, and **pose staleness** — a grasped part carried without
   being re-observed drifts to `EXTRAPOLATING` (dead-reckoned from arm FK), and an
   observation re-anchors it to `FRESH`. This is the calibration layer's staleness idea
   (§4/`calibration/`) applied to 6-DoF pose. Camera-agnostic: fed by simulated
   observations now, a real fiducial / CAD-pose estimator later.
7. **Verify** — every placed part's tracked pose is checked against the CAD-nominal
   within tolerance. The CAD file closes the loop.

**Repo boundary.** Motion *execution* on the real leader/follower arms lives in the
`so101-lab` hardware repo (lerobot + Placo); `software-mfg` owns the design-time world
model, planning, CAM, and verification, and **composes `so101-lab` by reference** (like
the wire bender) — it consumes live arm pose, it does not fork the driver layer. The
seam is `bridge/` (declared in `cells.yaml`): it invokes so101-lab's own Placo kinematics
for motion when the cell's venv+URDF are provisioned, and falls back to the built-in
positional IK otherwise — reporting which backend is live (LIVE vs SIM), the same honesty
as the calibration staleness stamp. The residual sim-world ↔ real-arm-base transform is a
hand-eye/base **calibration** item (identity until measured).

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
