#!/usr/bin/env python3
"""step_recognize.py — recover a featuretree IR from a STEP part, verified by re-emit.

Wraps the featuretree cell's recognizer (../featuretree/step_recognize.py, composed by
reference). A STEP file is a dumb B-rep with no feature tree, so this INFERS one for the
2.5D-prismatic class (a flat base extrusion + circular through/blind holes) by classifying
faces with the OCCT kernel, then SELF-VERIFIES by re-emitting the IR through build123d and
comparing volume + bbox. In-scope parts come back VERIFIED (Δ≈0) and can then drive the
editable FreeCAD/Onshape tree; anything else (fillets, bosses, revolves/lofts/sweeps) is
flagged PARTIAL so you fall back to importing the STEP as one solid — it never fakes a tree.

    python scripts/step_recognize.py [part.step ...]   # default: a VERIFIED + a PARTIAL demo
    python scripts/step_recognize.py --check            # gate (delegates to the cell selftest)
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))
from freecad_cmd import featuretree_root  # noqa: E402

sys.path.insert(0, str(featuretree_root()))
import step_recognize as sr  # noqa: E402  (the featuretree recognizer)

EXPORTS = ROOT / "exports"


def demo(paths):
    if not paths:
        print("no STEP files found — run `make parts` to populate exports/*.step, or pass paths")
        return 0
    for p in paths:
        try:
            spec, rep = sr.recognize(p)
        except Exception as e:
            print(f"UNSUPPORTED {Path(p).name:24} {e}")
            continue
        v = rep.get("verified")
        tag = "VERIFIED " if v else "PARTIAL  "
        tree = " -> ".join(f["name"] for f in spec["features"])
        print(f"{tag}{Path(p).name:24} vol STEP={rep.get('vol_orig')} IR={rep.get('vol_ir')} "
              f"Δ{rep.get('dvol_pct')}%  (thru={rep['through_holes']} blind={rep['blind_holes']})")
        print(f"          tree: {tree}")
        for w in rep["warnings"]:
            print("          !", w)
        if not v:
            print("          => fall back to importing this STEP as one solid")
    return 0


def main():
    args = sys.argv[1:]
    if "--check" in args:
        return sr.selftest()
    paths = [a for a in args if not a.startswith("-")]
    if not paths:                       # representative default: one in-scope, one out-of-scope
        paths = [str(EXPORTS / n) for n in ("example_plate.step", "coupling_tool_side.step")
                 if (EXPORTS / n).exists()]
    return demo(paths)


if __name__ == "__main__":
    sys.exit(main())
