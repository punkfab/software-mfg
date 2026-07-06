"""assembly.py — the CAD assembly reference: the target poses + mates a build aims for.

The STEP/CAD assembly is both the PLAN (where each part goes) and the CHECK (did the
placed part land where the CAD says). Here a small sample assembly, poses in the workcell
datum frame; swap in a real partcad/STEP assembly (parts + transforms) later. `verify()`
compares tracked poses to the CAD-nominal within tolerance — the assembly-level gate.
"""

from .world import PLACED, Pose, WorldModel

# A sample assembly in the workcell datum frame (metres, datum-relative). Each part
# names its CAD source and the pose the assembly wants it at.
SAMPLE = {
    "name": "bracket_weldment",
    "parts": [
        {"name": "bracket", "cad_ref": "parts/example_plate.py", "nominal": Pose((0.0, 0.0, 0.0))},
        {"name": "staple", "cad_ref": "wirebender:staple", "nominal": Pose((0.0, 0.010, 0.006))},
    ],
    "mates": [("staple", "bracket", "glue_seam")],   # staple glued to bracket along a seam
}


def load(assembly=SAMPLE) -> WorldModel:
    w = WorldModel()
    for p in assembly["parts"]:
        w.add(p["name"], p["cad_ref"], p["nominal"])
    return w


def verify(world: WorldModel, tol_mm=1.5, tol_deg=3.0):
    """Every part must be PLACED and within tolerance of its CAD-nominal pose."""
    results, ok = {}, True
    for name, p in world.parts.items():
        if p.grasp_state != PLACED:
            results[name] = {"placed": False}
            ok = False
            continue
        e = world.place_error(name)
        good = e["trans_mm"] <= tol_mm and e["ang_deg"] <= tol_deg
        results[name] = {"placed": True, "ok": good, **e}
        ok = ok and good
    return ok, results
