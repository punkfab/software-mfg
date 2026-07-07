# software-mfg

**Software-defined manufacturing & assembly** — a harness that builds *physical*
harnesses. The same way an AI coding harness defines, runs, and iterates on
software, this project defines parts and assemblies as code, simulates them,
and orchestrates multi-step / multi-process physical builds (CNC wire bending,
shearing, 3D printing, heat-set inserts) using small robot arms — improving the
design and the process automatically as it solidifies.

- **North star (Manufacturing for Design):** [NORTH_STAR.md](./NORTH_STAR.md)
- **Clean-manufacturing thesis (the green case):** [CLEAN_MFG.md](./CLEAN_MFG.md)
- **Vision & architecture:** [CONCEPT.md](./CONCEPT.md)
- **Roadmap:** [PLAN.md](./PLAN.md)
- **Process design space (survey):** [PROCESS_SURVEY.md](./PROCESS_SURVEY.md)
- **Mobile base + scanning surveys:** [MOBILE_BASE.md](./MOBILE_BASE.md) · [SCANNING_SURVEY.md](./SCANNING_SURVEY.md)

## Core idea

Three coupled loops, each keeping the next honest:

1. **Design round-trip** — a part is a declarative feature-IR that emits an
   *editable* FreeCAD/Onshape tree and folds human edits back by feature name
   (`featuretree/`). Not a dead STEP hand-off.
2. **Sim-CI** — design → simulate (MuJoCo) → verdict. Fast, cheap, repeatable;
   ~95% of the iteration happens here. Every `*-check` gate is one test.
3. **Reality calibration** — a physical build measures what the sim got wrong and
   writes the correction back into the model (`calibration/`). This is the slow
   outer loop that *corrects the simulator's lies*.

The catch sim-CI can't escape on its own: you **test a simulation** but **ship a
physical part**. A green sim only means something where its parameters have been
anchored to reality — so the sim is treated as a **cache of reality**, and the
calibration layer is its invalidation signal. Everything — parts, assemblies,
process steps, and the calibrated parameters themselves — is design-as-code,
versioned and regenerable.

## Layout

```
parts/         # build123d part scripts authored here (each exposes `part`)
assemblies/    # partcad assemblies-as-code (composition + positions)
sim/           # cells: workcell, printer, wirebender, press, toolchanger, SO-101, foil former + LOM
               #   + interference.py: solid part interference / swept-collision checking
               #   + station.py: material-handling floor (mobile carriers, routing, work envelope)
featuretree/   # feature-IR -> editable FreeCAD/Onshape tree, round-tripped by name
orchestration/ # op-graph + scheduler + CAM toolpath (bead/insert/pick-place) + assemble driver
calibration/   # the reality leg: calibrated parameter vector + staleness stamp + writeback
tracking/      # world model: CAD-referenced part-pose tracking + staleness + verify
bridge/        # compose so101-lab by reference: real Placo motion, built-in IK fallback
scripts/       # one check_*.py gate + one *_demo.py renderer per cell/subsystem
exports/       # generated STEP / STL / 3MF + renders (gitignored); cells/ for cell output
cells.yaml     # external machine cells composed by reference (e.g. ../wirebender)
```

## Setup

build123d and mujoco are the core deps. With `uv`:

```bash
uv sync                        # or: pip install -e .
python scripts/check_parts.py  # regenerate + validate local parts -> exports/
python scripts/sync_cells.py   # pull geometry from external cells -> exports/cells/
```

Or use the Makefile (`make help` lists all targets):

```bash
make sim       # open the SO-101 in the live interactive viewer (needs a display)
make render    # headless scripted-motion video -> exports/renders/
make check     # run every validation gate (parts + cells + model + calibration)
make calib     # walk the calibration round-trip (staleness stamp + measurement writeback)
```

## Composing cells (don't fork them)

A machine that already lives as its own parts-as-code repo (the wire bender at
`../wirebender`) is consumed **by reference**, declared in `cells.yaml`.
`sync_cells.py` invokes the cell's *own* interpreter/CAD to emit STEP+STL, then
re-imports the STEP here as an integrity gate. The cell repo is the single source
of truth and is **never modified** (sync writes no bytecode into it).

Recorded toolchain (Phase 0): Python 3.12.9, build123d 0.10.0, mujoco 3.9.0,
partcad 0.7.135.

## Round-trip with reality (calibration)

The outer loop is mechanized in `calibration/` — see
[calibration/README.md](./calibration/README.md) for detail. In short:

- The real source of truth isn't geometry or code, it's the **calibrated parameter
  vector** (`calibration/store.json`): each parameter is a *value + σ + the operating
  point it was measured at + the envelope it's trusted over + which physical build
  last confirmed it*. Sims read it (`sim/press_cell.py` pulls its press force and
  seat depth from the store); measurements write it.
- Every sim result carries a **staleness stamp** — `FRESH` (trust it), `STALE`
  (too many design iterations since reality confirmed it → re-anchor), or
  `EXTRAPOLATING` (ran outside where the model was ever validated → green is a
  guess). Staleness is counted in **builds-since-anchor**, not wall-clock, because
  the design target moves.
- A physical measurement is ingested as a **reviewable diff** (`ParamDiff`: old →
  new, the residual the sim got wrong, updated σ), not a silent overwrite — the
  same review-then-commit discipline as the featuretree design round-trip.
- `make calib-check` is a gate (part of `make check`): it fails if a sim leans on
  parameters reality hasn't confirmed lately, or if the −2σ edge of a calibrated
  value would miss its requirement. Green means *in-tolerance with margin*, not
  merely at nominal.

## The parts convention

Each file in `parts/` (not `_`-prefixed) defines a module-level variable `part`
holding a build123d solid. `scripts/check_parts.py` imports each, asserts it is a
single watertight solid with positive volume, and exports STEP + STL to
`exports/`. This is the Phase 0 geometry CI check.
