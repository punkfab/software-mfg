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


def sketch(name, plane="XY", circles=(), rects=(), on=None):
    """circles: [(cx, cy, r), ...]   rects: [(w, h, cx, cy), ...]
    on: None -> XY plane; or a face QUERY {"face_of": feature, "side": "top"|"bottom"}
    (coords stay global; the emitter transforms them into the face's local frame)."""
    return {"kind": "sketch", "name": name, "plane": plane, "on": on,
            "circles": [list(c) for c in circles], "rects": [list(r) for r in rects]}


def pad(name, sketch, length, symmetric=False):
    return {"kind": "pad", "name": name, "sketch": sketch,
            "length": length, "symmetric": symmetric}


def pocket(name, sketch, through=True, length=None):
    return {"kind": "pocket", "name": name, "sketch": sketch,
            "through": through, "length": length}


def fillet(name, radius, select):
    """Round edges chosen by a QUERY (resolved against live geometry at build time,
    never stored kernel edge ids). select e.g. {"circles": "top_outer"}."""
    return {"kind": "fillet", "name": name, "radius": radius, "select": select}


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
        if f["kind"] == "fillet" and "radius" in p:
            f["radius"] = p["radius"]
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


def coupling_plate():
    """The real tool-changer coupling blank (cf. parts/_coupling.py): a Ø50x6 disc
    with a Ø12 central bore + 3 M3 mounting holes, and a filleted top rim.

    The fillet selects the disc's top outer edge by QUERY, not by a stored edge id —
    so it survives edits/rebuilds. This is a real project part, round-tripped.
    """
    import math
    bolts = [(round(21 * math.cos(math.radians(a)), 4),
              round(21 * math.sin(math.radians(a)), 4)) for a in (30, 150, 270)]
    return part(
        "coupling_plate",
        sketch("disc_outline", "XY", circles=[(0, 0, 25)]),
        pad("disc", "disc_outline", length=6),
        fillet("rim_round", radius=1.0, select={"circles": "top_outer"}),
        sketch("holes", "XY", circles=[(0, 0, 6)] + [(x, y, 1.7) for x, y in bolts]),
        pocket("drill", "holes", through=True),
        # counterbore recess around the bore, drilled from the TOP FACE (face-attached
        # sketch — the face is chosen by query, then coords map into its local frame)
        sketch("recess_sk", circles=[(0, 0, 9)], on={"face_of": "drill", "side": "top"}),
        pocket("recess", "recess_sk", through=False, length=1.5),
    )


SAMPLES = {"plate": sample_plate, "coupling_plate": coupling_plate}
