#!/usr/bin/env python3
"""Emit a FreeCAD .FCStd feature tree from the IR, via FreeCAD's own interpreter.

    python scripts/freecad_gen.py            # generates the sample plate
Writes exports/freecad/<part>.FCStd and prints the resulting feature tree.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "featuretree"))
import ir as IR  # noqa: E402
from freecad import run_in_freecad  # noqa: E402

OUT = ROOT / "exports" / "freecad"
FC_BUILD = ROOT / "featuretree" / "fc_build.py"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    sample = sys.argv[1] if len(sys.argv) > 1 else "plate"
    spec = IR.SAMPLES[sample]()
    ir_json = OUT / f"{spec['name']}.ir.json"
    fcstd = OUT / f"{spec['name']}.FCStd"
    ir_json.write_text(json.dumps(spec, indent=2))

    proc = run_in_freecad(str(FC_BUILD),
                          {"FC_IR": ir_json, "FC_OUT": fcstd, "FC_LIBDIR": ROOT / "featuretree"})
    line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT:")), None)
    if line is None:
        print("FreeCAD emit failed:\n", proc.stdout[-800:], proc.stderr[-800:])
        return 1
    result = json.loads(line[len("RESULT:"):])

    print(f"emitted {fcstd.relative_to(ROOT)}  (open in FreeCAD — tree on the left)")
    print(f"volume: {result['volume']} mm^3   (build123d example_plate = 11497.3)")
    print("feature tree:")
    for label, typeid in result["tree"]:
        print(f"  {label:14} {typeid}")
    print("editable params:", json.dumps(result["params"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
