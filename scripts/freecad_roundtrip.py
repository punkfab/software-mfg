#!/usr/bin/env python3
"""Round-trip a part: IR -> FreeCAD tree -> human edits a param -> back into the IR.

    python scripts/freecad_roundtrip.py [plate|coupling_plate]

Demonstrates the inbound direction neutral feature files can't do: a human changes
a parameter in the FreeCAD doc and — because every feature carries its IR name as
its Label — we read the change back, update the IR, and regenerate. For
coupling_plate the edited param is the *fillet radius*, whose edge was chosen by a
query (not a stored edge id), proving query-based features round-trip too.
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
EDITS = {                       # sample -> (feature, param, new value)
    "plate": ("body", "length", 18.0),
    "coupling_plate": ("rim_round", "radius", 2.5),
}


def run_fc(script, env):
    proc = run_in_freecad(str(LIB / script), {**env, "FC_LIBDIR": LIB})
    line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT:")), None)
    if line is None:
        raise RuntimeError(f"{script} failed:\n{proc.stdout[-700:]}\n{proc.stderr[-700:]}")
    return json.loads(line[len("RESULT:"):])


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    sample = sys.argv[1] if len(sys.argv) > 1 else "plate"
    feat, key, newval = EDITS[sample]
    spec = IR.SAMPLES[sample]()
    ir_json, fcstd = OUT / "rt.ir.json", OUT / "rt.FCStd"
    problems = []

    ir_json.write_text(json.dumps(spec))
    r0 = run_fc("fc_build.py", {"FC_IR": ir_json, "FC_OUT": fcstd})
    print(f"1. IR -> tree '{sample}': vol={r0['volume']}  {feat}.{key}={r0['params'][feat][key]}")

    r2 = run_fc("fc_read.py", {"FC_IN": fcstd, "FC_EDIT": json.dumps({feat: {key: newval}})})
    print(f"2. human edits {feat}.{key} -> {r2['params'][feat][key]}  vol={r2['volume']}")
    if r2["params"][feat][key] != newval:
        problems.append("human edit did not take in the FreeCAD doc")

    IR.update_from_freecad(spec, r2["params"])
    ir_json.write_text(json.dumps(spec))
    r3 = run_fc("fc_build.py", {"FC_IR": ir_json, "FC_OUT": OUT / "rt2.FCStd"})
    print(f"3. IR updated + regenerated: {feat}.{key}={r3['params'][feat][key]}  vol={r3['volume']}")
    if r3["params"][feat][key] != newval:
        problems.append("edit did not flow back into the IR")
    if abs(r3["volume"] - r2["volume"]) > 1.0:
        problems.append(f"regenerated vol {r3['volume']} != edited vol {r2['volume']}")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print(f"\nPASS: '{sample}' round-trip closed (IR -> tree -> human edit -> IR -> regenerate)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
