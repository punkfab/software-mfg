"""fc_build.py — emit a FreeCAD .FCStd feature tree from the IR.

RUNS UNDER freecadcmd (FreeCAD's Python 3.11), not the project interpreter.
Inputs via env: FC_IR (ir json path), FC_OUT (.FCStd path). (Paths can't be argv —
freecadcmd opens path args as documents.)
Prints a line "RESULT:" + json with the tree, final volume, and as-built params.

Each feature becomes a native PartDesign object with Label = the IR name, so a
human edit in FreeCAD can be matched back by name (see fc_read.py).
"""

import json
import os
import sys

import FreeCAD as App
import Part

sys.path.insert(0, os.environ.get("FC_LIBDIR", os.path.dirname(os.path.abspath(__file__))))
import fc_common  # noqa: E402


def _add_rect(sk, w, h, cx, cy):
    hw, hh = w / 2.0, h / 2.0
    pts = [App.Vector(cx - hw, cy - hh, 0), App.Vector(cx + hw, cy - hh, 0),
           App.Vector(cx + hw, cy + hh, 0), App.Vector(cx - hw, cy + hh, 0)]
    for i in range(4):
        sk.addGeometry(Part.LineSegment(pts[i], pts[(i + 1) % 4]), False)


def build(spec, out_path):
    doc = App.newDocument(spec["name"])
    body = doc.addObject("PartDesign::Body", "Body")
    sketches = {}

    for f in spec["features"]:
        kind = f["kind"]
        if kind == "sketch":
            if f["plane"] != "XY":
                raise ValueError("v0 supports only XY-plane sketches")
            sk = body.newObject("Sketcher::SketchObject", f["name"])
            sk.Label = f["name"]
            for (cx, cy, r) in f["circles"]:
                sk.addGeometry(Part.Circle(App.Vector(cx, cy, 0), App.Vector(0, 0, 1), r), False)
            for (w, h, cx, cy) in f["rects"]:
                _add_rect(sk, w, h, cx, cy)
            sketches[f["name"]] = sk
        elif kind == "pad":
            p = body.newObject("PartDesign::Pad", f["name"])
            p.Label = f["name"]
            p.Profile = sketches[f["sketch"]]
            p.Length = f["length"]
            p.Midplane = bool(f["symmetric"])
        elif kind == "pocket":
            p = body.newObject("PartDesign::Pocket", f["name"])
            p.Label = f["name"]
            p.Profile = sketches[f["sketch"]]
            if f["through"]:
                p.Type = "ThroughAll"
                p.Midplane = True            # cut both ways -> robust without a face attach
            else:
                p.Length = f["length"]
        else:
            raise ValueError(f"unknown feature kind: {kind}")
        doc.recompute()

    doc.recompute()
    doc.saveAs(out_path)
    return fc_common.result(doc)


# freecadcmd execs this file but not as __main__, so run at top level.
_spec = json.load(open(os.environ["FC_IR"]))
_result = build(_spec, os.environ["FC_OUT"])
print("RESULT:" + json.dumps(_result))
