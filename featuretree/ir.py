"""ir.py — a neutral, named feature-tree IR (our CAD DSL).

The operation-preserving representation that no neutral *file* format provides:
a declarative, ordered list of named features with named parameters. It is the
single source of truth; per-target emitters (FreeCAD now; Onshape FeatureScript,
Fusion API, a SolidWorks macro later) translate it into each tool's *native*
feature vocabulary, so the tree shows up editable on the left.

Design choices that make round-trip tractable:
  * every feature has a stable, human-meaningful `name` (→ the target object's
    Label) so a human edit can be matched back by name, not kernel edge id;
  * parameters are named values, not positions in a blob;
  * geometry is referenced symbolically (a feature name + plane), never by
    unstable kernel edge/face ids — the wall that kills neutral feature files.

v0 scope: sketches (rectangles + circles) on principal planes, Pad, Pocket
(through). Face-attached sketches and edge selectors (fillets) are the next,
harder step (the topological-naming problem) and are deliberately deferred.

The IR is plain JSON-able dicts so it crosses cleanly into FreeCAD's separate
Python (3.11) interpreter.
"""


def sketch(name, plane="XY", circles=(), rects=()):
    """circles: [(cx, cy, r), ...]   rects: [(w, h, cx, cy), ...]"""
    return {"kind": "sketch", "name": name, "plane": plane,
            "circles": [list(c) for c in circles], "rects": [list(r) for r in rects]}


def pad(name, sketch, length, symmetric=False):
    return {"kind": "pad", "name": name, "sketch": sketch,
            "length": length, "symmetric": symmetric}


def pocket(name, sketch, through=True, length=None):
    return {"kind": "pocket", "name": name, "sketch": sketch,
            "through": through, "length": length}


def part(name, *features):
    return {"name": name, "features": list(features)}


def update_from_freecad(spec, params):
    """Flow human edits (read back from a .FCStd, keyed by feature name) into the IR."""
    for f in spec["features"]:
        p = params.get(f["name"])
        if not p:
            continue
        if f["kind"] in ("pad", "pocket") and "length" in p and f.get("length") is not None:
            f["length"] = p["length"]
        if f["kind"] == "sketch" and "radii" in p:
            for i, r in enumerate(p["radii"]):
                if i < len(f["circles"]):
                    f["circles"][i][2] = r
    return spec


def sample_plate():
    """A 40x30x10 plate with an 8 mm through hole — mirrors parts/example_plate.py.

    FreeCAD volume should match build123d's example_plate (11497 mm^3), proving
    the IR drives an equivalent model through a different kernel.
    """
    return part(
        "plate",
        sketch("outline", "XY", rects=[(40, 30, 0, 0)]),
        pad("body", "outline", length=10),
        sketch("hole_sketch", "XY", circles=[(0, 0, 4)]),
        pocket("hole", "hole_sketch", through=True),
    )
