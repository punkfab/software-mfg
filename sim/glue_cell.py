"""glue_cell.py — forward model for the hot-glue applicator tool.

Gluing has almost no process force (unlike the shear/press) — the physics that
matters is THERMAL (melt-ready time) and EXTRUSION (positive displacement: pushing
the solid stick displaces molten glue out the nozzle), plus the adhesive's OPEN and
SET times, which drive multi-arm coordination (the other arm must hold the joint
until it sets). All calibrated (calibration/store.json), unanchored → PREDICTION.

The "program" is a bead PATH (a polyline the nozzle traces) — the direct analog of
the wire bender's bend program or a toolpath.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "parts"))
from calibration import param  # noqa: E402
from _glue import NOZZLE_D, STICK_D  # noqa: E402

MELT_TEMP_C = param("glue_melt_temp_c", 150.0)
HEATUP_TAU_S = param("glue_heatup_tau_s", 35.0)
BEAD_WIDTH_MM = param("glue_bead_width_mm", 3.0)
OPEN_TIME_S = param("glue_open_time_s", 15.0)
SET_TIME_S = param("glue_set_time_s", 60.0)

AMBIENT_C = 20.0
READY_BAND_C = 2.0        # "ready" when within this of melt temp
BEAD_SPEED_MM_S = 15.0    # nozzle travel while extruding (bead_width op point)

# bead PATHs (mm): the nozzle traces these; the join geometry the tool produces.
PROGRAMS = {
    "seam": [(0.0, 0.0), (60.0, 0.0)],                                   # a 60 mm line
    "perimeter": [(0, 0), (40, 0), (40, 25), (0, 25), (0, 0)],           # a 40x25 rectangle
}


def ready_time_s(ambient_c=AMBIENT_C):
    """Cold-start time for the melt cartridge to reach melt temperature (first-order)."""
    span = MELT_TEMP_C - ambient_c
    return HEATUP_TAU_S * math.log(max(span, READY_BAND_C) / READY_BAND_C)


def extrude_ratio():
    """mm of bead laid per mm of stick fed (positive displacement, volume-conserving).
    Bead modelled as a half-round of width BEAD_WIDTH_MM."""
    stick_area = math.pi * (STICK_D / 2) ** 2
    bead_area = 0.5 * math.pi * (BEAD_WIDTH_MM / 2) ** 2
    return stick_area / bead_area


def path_length_mm(path):
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1))


def apply_bead(program="seam"):
    """Lay a bead along a path. Returns the bead geometry + the coordination timing
    (heat is a pre-step; apply is fast; the joint must then be HELD until it sets)."""
    path = PROGRAMS[program]
    length = path_length_mm(path)
    feed_mm = length / extrude_ratio()
    apply_s = length / BEAD_SPEED_MM_S
    return {
        "program": program,
        "bead_length_mm": round(length, 1),
        "bead_width_mm": round(BEAD_WIDTH_MM, 2),
        "stick_feed_mm": round(feed_mm, 2),
        "apply_s": round(apply_s, 2),
        "ready_s": round(ready_time_s(), 1),      # cold-start heat (pre-step, amortized)
        "open_s": round(OPEN_TIME_S, 1),          # workable window after laydown
        "set_s": round(SET_TIME_S, 1),            # must be held this long
        "hold_window_s": round(apply_s + SET_TIME_S, 1),   # 2nd arm commitment
        "points": path,
    }


if __name__ == "__main__":
    r = apply_bead("perimeter")
    print(f"glue_cell: {r['program']} bead {r['bead_length_mm']}mm @ {r['bead_width_mm']}mm wide, "
          f"feed {r['stick_feed_mm']}mm | ready {r['ready_s']}s, apply {r['apply_s']}s, "
          f"hold {r['hold_window_s']}s (set {r['set_s']}s)")
