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
parts/         # build123d part scripts (each exposes a module-level `part`)
assemblies/    # partcad assemblies-as-code (composition + positions)
sim/           # MuJoCo MJCF models + scenes
orchestration/ # operation-graph + scheduler (later phases)
scripts/       # tooling — e.g. check_parts.py (regenerate + validate geometry)
exports/       # generated STEP / STL / 3MF (gitignored)
```

## Setup

build123d and mujoco are the core deps. With `uv`:

```bash
uv sync                       # or: pip install -e .
python scripts/check_parts.py # regenerate + validate every part -> exports/
```

Recorded toolchain (Phase 0): Python 3.12.9, build123d 0.10.0, mujoco 3.9.0,
partcad 0.7.135.

## The parts convention

Each file in `parts/` (not `_`-prefixed) defines a module-level variable `part`
holding a build123d solid. `scripts/check_parts.py` imports each, asserts it is a
single watertight solid with positive volume, and exports STEP + STL to
`exports/`. This is the Phase 0 geometry CI check.
