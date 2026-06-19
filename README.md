# software-mfg

**Software-defined manufacturing & assembly** — a harness that builds *physical*
harnesses. The same way an AI coding harness defines, runs, and iterates on
software, this project defines parts and assemblies as code, simulates them,
and orchestrates multi-step / multi-process physical builds (CNC wire bending,
shearing, 3D printing, heat-set inserts) using small robot arms — improving the
design and the process automatically as it solidifies.

- **Vision & architecture:** [CONCEPT.md](./CONCEPT.md)
- **Roadmap:** [PLAN.md](./PLAN.md)

## Core idea

Two coupled loops: a **fast inner loop in simulation** (MuJoCo) does ~95% of the
iteration; a **slow outer loop in the physical world** sparsely verifies winners
and corrects the simulator's lies. Everything — parts, assemblies, process steps
— is design-as-code, versioned and regenerable.

## Layout

```
parts/         # build123d part scripts authored here (each exposes `part`)
assemblies/    # partcad assemblies-as-code (composition + positions)
sim/           # MuJoCo MJCF models + scenes
orchestration/ # operation-graph + scheduler (later phases)
scripts/       # check_parts.py (local parts) + sync_cells.py (external cells)
exports/       # generated STEP / STL / 3MF (gitignored); cells/ for cell output
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
make check     # run every validation gate (parts + cells + model)
```

## Composing cells (don't fork them)

A machine that already lives as its own parts-as-code repo (the wire bender at
`../wirebender`) is consumed **by reference**, declared in `cells.yaml`.
`sync_cells.py` invokes the cell's *own* interpreter/CAD to emit STEP+STL, then
re-imports the STEP here as an integrity gate. The cell repo is the single source
of truth and is **never modified** (sync writes no bytecode into it).

Recorded toolchain (Phase 0): Python 3.12.9, build123d 0.10.0, mujoco 3.9.0,
partcad 0.7.135.

## The parts convention

Each file in `parts/` (not `_`-prefixed) defines a module-level variable `part`
holding a build123d solid. `scripts/check_parts.py` imports each, asserts it is a
single watertight solid with positive volume, and exports STEP + STL to
`exports/`. This is the Phase 0 geometry CI check.
