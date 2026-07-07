#!/usr/bin/env python3
"""Gate: solid part interference checking is sound — it catches real collisions, clears
real gaps, and NEVER silently passes a mesh it couldn't test.

Positive and negative controls on real STL exports (mm):
  - static overlap  -> flagged, exact volume + interpenetration depth
  - static gap      -> clear, min-separation reported
  - swept plunge    -> clear while high, FOULs when the tool descends into the part
  - open mesh       -> non-watertight part is tested via a conservative hull and marked
                       `approximate` (fail-safe): it must never report a phantom clearance
  - operations      -> insert/pick_place toolpaths produce sweepable, well-formed paths
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))
sys.path.insert(0, str(ROOT / "orchestration"))
import interference as ix  # noqa: E402
import toolpath as tp  # noqa: E402

PLATE = str(ROOT / "exports" / "example_plate.stl")     # 40x30x10mm, watertight
TOOL = str(ROOT / "exports" / "glue_body.stl")          # watertight, grows +z from base
OPEN_PART = str(ROOT / "exports" / "glue_nozzle_bracket.stl")   # non-watertight


def main() -> int:
    problems = []

    # 1. STATIC positive control — two plates overlapping 50% in x
    s = (ix.Scene().place("A", PLATE, ix.pose_matrix_mm((0, 0, 0)))
                   .place("B", PLATE, ix.pose_matrix_mm((20, 0, 0))))
    rep = s.check()
    if rep["ok"] or not rep["interferences"]:
        problems.append("static overlap not detected")
    else:
        h = rep["interferences"][0]
        if h["approximate"]:
            problems.append("watertight overlap wrongly flagged approximate")
        if not (h["volume_mm3"] > 100 and h["depth_mm"] > 1):
            problems.append(f"overlap metrics implausible: {h}")

    # 2. STATIC negative control — 1mm gap: clear, and the near-miss is reported
    s2 = (ix.Scene().place("A", PLATE, ix.pose_matrix_mm((0, 0, 0)))
                    .place("B", PLATE, ix.pose_matrix_mm((41, 0, 0))))
    rep2 = s2.check()
    if not rep2["ok"]:
        problems.append("1mm gap wrongly reported as interference")
    near = [c for c in rep2["clearances"] if abs(c["clearance_mm"] - 1.0) < 0.2]
    if not near:
        problems.append(f"1mm clearance not reported: {rep2['clearances']}")

    # 3. SWEEP — a plunge: tool descends from clear (z high) into the plate (z low)
    scene = ix.Scene().place("plate", PLATE, ix.pose_matrix_mm((0, 0, 0)))
    high = [(0.0, 0.0, z / 1000.0) for z in np.linspace(60, 40, 6)]   # tool base 40-60mm up
    plunge = [(0.0, 0.0, z / 1000.0) for z in np.linspace(40, -15, 12)]  # descends into plate
    sw_hi = ix.sweep(TOOL, ix.transforms_along(high), scene)
    sw_lo = ix.sweep(TOOL, ix.transforms_along(plunge), scene)
    if not sw_hi["ok"]:
        problems.append(f"tool clear above the part reported as foul: {sw_hi['first_hit']}")
    if sw_hi["pruned"] == 0:
        problems.append("broad phase pruned nothing on a clearly-separated sweep")
    if sw_lo["ok"] or not sw_lo["first_hit"]:
        problems.append("plunge into the part not detected")
    elif sw_lo["first_hit"]["part"] != "plate":
        problems.append(f"plunge hit the wrong body: {sw_lo['first_hit']}")

    # 4. OPEN-MESH fail-safe — a non-watertight part that overlaps must HIT and be flagged
    #    approximate; it must never silently report clear.
    so = ix.Scene().place("plate", PLATE, ix.pose_matrix_mm((0, 0, 0)))
    sw_open = ix.sweep(OPEN_PART, ix.transforms_along([(0, 0, 0.0)]), so)
    if sw_open["ok"]:
        problems.append("SILENT MISS: open mesh overlapping the plate reported clear")
    if not sw_open["approximate"]:
        problems.append("open mesh not flagged approximate (fail-safe contract broken)")

    # 5. OPERATIONS — the new toolpath generators are well-formed + sweepable
    ins = tp.insert_toolpath((0.30, 0.0, 0.10), approach_mm=25.0)
    if ins.poses[0][2] <= ins.poses[-1][2]:
        problems.append("insert_toolpath should descend to the seat")
    pp = tp.pick_place_toolpath((0.30, 0.0, 0.05), (0.30, 0.15, 0.05), safe_z_mm=40.0)
    if max(p[2] for p in pp.poses) < 0.05 + 0.039:
        problems.append("pick_place_toolpath should lift to the safe plane")

    # --- report ---
    h = rep["interferences"][0]
    print(f"static overlap: {h['volume_mm3']}mm³, depth {h['depth_mm']}mm (exact) | "
          f"gap {near[0]['clearance_mm']}mm clear")
    print(f"sweep plunge: clear above ({sw_hi['pruned']} pruned) -> FOUL @wp"
          f"{sw_lo['first_hit']['index']}/{sw_lo['waypoints']} "
          f"({sw_lo['first_hit']['volume_mm3']}mm³ into {sw_lo['first_hit']['part']})")
    print(f"open mesh: hit={not sw_open['ok']} approximate={sw_open['approximate']} "
          f"(conservative hull — fail-safe, never a silent clear)")
    print(f"operations: insert descends {ins.poses[0][2]*1000:.0f}->{ins.poses[-1][2]*1000:.0f}mm, "
          f"pick_place lifts to {max(p[2] for p in pp.poses)*1000:.0f}mm safe plane")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: interference checking catches collisions, clears gaps, and fails safe on open meshes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
