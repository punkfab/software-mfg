#!/usr/bin/env python3
"""Regenerate + validate every part script. Phase 0 geometry CI check.

Convention: each parts/*.py (not `_`-prefixed) exposes a module-level `part`
(a build123d solid). For each one this script:
  1. imports the module (re-runs the parametric build),
  2. asserts it is a single solid with positive volume and valid geometry,
  3. exports STEP + STL to exports/.

Exit code is non-zero if any part fails, so it can gate CI.
"""

import importlib.util
import sys
from pathlib import Path

from build123d import export_step, export_stl

ROOT = Path(__file__).resolve().parent.parent
PARTS = ROOT / "parts"
OUT = ROOT / "exports"


def load_part(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return getattr(mod, "part", None)


def validate(obj) -> list[str]:
    problems: list[str] = []
    solids = obj.solids()
    if len(solids) != 1:
        problems.append(f"expected 1 solid, got {len(solids)}")
    if obj.volume <= 0:
        problems.append(f"non-positive volume {obj.volume:.3f}")
    iv = getattr(obj, "is_valid", True)         # bool in some build123d builds, callable in others
    if not (iv() if callable(iv) else iv):
        problems.append("OCC reports invalid geometry")
    return problems


def main() -> int:
    OUT.mkdir(exist_ok=True)
    files = sorted(p for p in PARTS.glob("*.py") if not p.name.startswith("_"))
    if not files:
        print("no part scripts found in parts/")
        return 0

    failures = 0
    for f in files:
        name = f.stem
        try:
            obj = load_part(f)
            if obj is None:
                print(f"FAIL {name}: no module-level `part`")
                failures += 1
                continue
            problems = validate(obj)
            if problems:
                print(f"FAIL {name}: {'; '.join(problems)}")
                failures += 1
                continue
            export_step(obj, str(OUT / f"{name}.step"))
            export_stl(obj, str(OUT / f"{name}.stl"))
            print(f"OK   {name}: vol={obj.volume:.1f} mm^3 -> {name}.step, {name}.stl")
        except Exception as e:  # noqa: BLE001 - report any build failure per-part
            print(f"FAIL {name}: {type(e).__name__}: {e}")
            failures += 1

    print(f"\n{len(files) - failures}/{len(files)} parts OK")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
