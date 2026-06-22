"""fc_common.py — shared FreeCAD-side helpers (runs under freecadcmd).

Side-effect free so both fc_build.py and fc_read.py can import it.
"""

import Part


def result(doc):
    """Extract the as-built tree, final volume, and editable params (keyed by Label)."""
    body = next(o for o in doc.Objects if o.TypeId == "PartDesign::Body")
    feats = [o for o in doc.Objects if o.TypeId.startswith(("PartDesign", "Sketcher"))]
    params = {}
    for o in feats:
        if o.TypeId in ("PartDesign::Pad", "PartDesign::Pocket"):
            params[o.Label] = {"length": round(float(o.Length), 4)}
            if o.TypeId == "PartDesign::Pocket":
                params[o.Label]["type"] = str(o.Type)
        elif o.TypeId == "Sketcher::SketchObject":
            radii = [round(g.Radius, 4) for g in o.Geometry if isinstance(g, Part.Circle)]
            if radii:
                params[o.Label] = {"radii": radii}
    return {
        "tree": [(o.Label, o.TypeId) for o in feats],
        "volume": round(float(body.Shape.Volume), 1),
        "params": params,
    }


def apply_edits(doc, edits):
    """Apply {label: {key: value}} edits to named features — a human dimension change.

    Supports pad/pocket 'length' and sketch 'radius' (first circle). Returns the
    labels actually changed.
    """
    changed = []
    by_label = {o.Label: o for o in doc.Objects}
    for label, kv in edits.items():
        o = by_label.get(label)
        if o is None:
            continue
        if "length" in kv and hasattr(o, "Length"):
            o.Length = float(kv["length"])
            changed.append(label)
        if "radius" in kv and o.TypeId == "Sketcher::SketchObject":
            for i, g in enumerate(o.Geometry):
                if isinstance(g, Part.Circle):
                    o.setRadius(i, float(kv["radius"]))
                    changed.append(label)
                    break
    doc.recompute()
    return changed
