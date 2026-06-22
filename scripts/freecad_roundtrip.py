#!/usr/bin/env python3
"""Round-trip: IR -> FreeCAD tree -> (human edits a dimension) -> back into the IR.

Demonstrates the inbound direction that neutral feature files can't do: a human
changes a parameter in the FreeCAD doc, and because every feature carries its IR
name as its Label, we read the change back and update the IR — then regenerate.

    python scripts/freecad_roundtrip.py
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "featuretree"))
import ir as IR  # noqa: E402
from freecad import run_in_freecad  # noqa: E402

LIB = ROOT / "featuretree"
OUT = ROOT / "exports" / "freecad"


def run_fc(script, env):
    proc = run_in_freecad(str(LIB / script), {**env, "FC_LIBDIR": LIB})
    line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT:")), None)
    if line is None:
        raise RuntimeError(f"{script} failed:\n{proc.stdout[-600:]}\n{proc.stderr[-600:]}")
    return json.loads(line[len("RESULT:"):])


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    spec = IR.sample_plate()
    ir_json, fcstd = OUT / "rt.ir.json", OUT / "rt.FCStd"
    problems = []

    # 1. IR -> FreeCAD tree
    ir_json.write_text(json.dumps(spec))
    r0 = run_fc("fc_build.py", {"FC_IR": ir_json, "FC_OUT": fcstd})
    print(f"1. emitted tree, vol={r0['volume']}  length(body)={r0['params']['body']['length']}")

    # 2. read back unchanged (recover params by name)
    r1 = run_fc("fc_read.py", {"FC_IN": fcstd})
    if r1["params"]["body"]["length"] != 10.0:
        problems.append("did not recover pad length on read-back")

    # 3. human edits the pad length 10 -> 18 in the FreeCAD doc
    r2 = run_fc("fc_read.py", {"FC_IN": fcstd, "FC_EDIT": json.dumps({"body": {"length": 18}})})
    print(f"2. human edits body length -> {r2['params']['body']['length']}  vol={r2['volume']}")
    if r2["params"]["body"]["length"] != 18.0 or r2["volume"] <= r0["volume"]:
        problems.append("human edit did not take / volume did not grow")

    # 4. flow the edit back into the IR, by feature name
    IR.update_from_freecad(spec, r2["params"])
    if spec["features"][1]["length"] != 18.0:
        problems.append("IR not updated from the FreeCAD edit")
    print(f"3. IR updated from FreeCAD: body length now {spec['features'][1]['length']}")

    # 5. regenerate from the updated IR — loop closed, geometry agrees
    ir_json.write_text(json.dumps(spec))
    r3 = run_fc("fc_build.py", {"FC_IR": ir_json, "FC_OUT": OUT / "rt2.FCStd"})
    print(f"4. regenerated from updated IR, vol={r3['volume']}  (matches edited {r2['volume']})")
    if abs(r3["volume"] - r2["volume"]) > 1.0:
        problems.append("regenerated volume disagrees with the edited doc")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("\nPASS: IR -> FreeCAD tree -> human edit -> IR -> regenerate (round-trip closed)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
