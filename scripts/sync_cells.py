#!/usr/bin/env python3
"""Materialize geometry from external manufacturing cells (compose, don't fork).

Reads cells.yaml and, for each declared part, runs the *cell's own* interpreter
on the cell's own CAD source to export STEP + STL into exports/cells/<cell>/.
Then re-imports the STEP under software-mfg's build123d as a cross-interpreter
integrity gate (single solid, positive volume).

This keeps the cell repo (e.g. ../wirebender) the single source of truth: we
never copy its CAD, we ask it to produce geometry. See cells.yaml / PLAN.md.

Exit code is non-zero if any part fails.
"""

import json
import os
import subprocess
import sys
from pathlib import Path

import yaml
from build123d import import_step

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = ROOT / "cells.yaml"
OUT = ROOT / "exports" / "cells"

# Runner executed inside the cell's interpreter. Args: module builder syspath_json out_step out_stl
CELL_RUNNER = r"""
import sys, os, json, importlib
module, builder, syspath_json, out_step, out_stl = sys.argv[1:6]
for p in reversed(json.loads(syspath_json)):
    sys.path.insert(0, os.path.abspath(p))
obj = getattr(importlib.import_module(module), builder)()
from build123d import export_step, export_stl
export_step(obj, out_step)
export_stl(obj, out_stl)
iv = obj.is_valid                       # bool in some build123d builds, callable in others
iv = iv() if callable(iv) else bool(iv)
print(json.dumps({"volume": round(obj.volume, 3), "solids": len(obj.solids()), "valid": iv}))
"""


def build_part(cell_name, cell, part_name, part) -> list[str]:
    """Run one part through its cell; return a list of problems (empty == OK)."""
    cell_path = (ROOT / cell["path"]).resolve()
    cell_py = cell_path / cell["python"]
    if not cell_py.exists():
        return [f"cell interpreter not found: {cell_py}"]

    dest = OUT / cell_name
    dest.mkdir(parents=True, exist_ok=True)
    out_step = dest / f"{part_name}.step"
    out_stl = dest / f"{part_name}.stl"

    proc = subprocess.run(
        [str(cell_py), "-c", CELL_RUNNER, part["module"], part["builder"],
         json.dumps(cell.get("syspath", ["."])), str(out_step), str(out_stl)],
        cwd=str(cell_path), capture_output=True, text=True,
        # Never write bytecode into the cell repo — keep the source tree untouched.
        env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
    )
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout).strip().splitlines()[-1:] or ["(no output)"]
        return [f"cell build failed: {tail[0]}"]

    try:
        metrics = json.loads(proc.stdout.strip().splitlines()[-1])
    except (ValueError, IndexError):
        return [f"could not parse cell metrics from: {proc.stdout!r}"]

    # Integrity gate: re-import the STEP under our own build123d.
    problems = []
    try:
        reimported = import_step(str(out_step))
        if len(reimported.solids()) != 1:
            problems.append(f"reimported STEP has {len(reimported.solids())} solids")
        if reimported.volume <= 0:
            problems.append(f"reimported STEP volume {reimported.volume:.3f} <= 0")
    except Exception as e:  # noqa: BLE001
        problems.append(f"STEP re-import failed: {type(e).__name__}: {e}")

    if not metrics.get("valid", False):
        problems.append("cell reports geometry not valid")

    if not problems:
        print(f"OK   {cell_name}/{part_name}: vol={metrics['volume']:.1f} mm^3 "
              f"solids={metrics['solids']} -> exports/cells/{cell_name}/{part_name}.step/.stl")
    return problems


def main() -> int:
    if not MANIFEST.exists():
        print("no cells.yaml — nothing to sync")
        return 0
    spec = yaml.safe_load(MANIFEST.read_text()) or {}
    cells = spec.get("cells", {})

    total = failures = 0
    for cell_name, cell in cells.items():
        for part_name, part in cell.get("parts", {}).items():
            total += 1
            problems = build_part(cell_name, cell, part_name, part)
            if problems:
                failures += 1
                print(f"FAIL {cell_name}/{part_name}: {'; '.join(problems)}")

    if total == 0:
        print("cells.yaml declares no parts")
        return 0
    print(f"\n{total - failures}/{total} cell parts synced")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
