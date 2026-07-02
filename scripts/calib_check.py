#!/usr/bin/env python3
"""Gate: the press sim's calibrated parameters are trustworthy.

A green sim only means something if the parameters it ran on are FRESH (recently
anchored to a physical build) and IN-ENVELOPE (not extrapolated), with margin across
their uncertainty band. This gate makes "green sim on stale calibration is a lie"
mechanical: it fails if the press cell is leaning on parameters reality hasn't
confirmed lately, or if the -2sigma edge of the calibrated press force would miss
the seating requirement.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from calibration import CalibrationStore, staleness  # noqa: E402

# The parameters the press sim consumes, and the op point it runs at.
PRESS_PARAMS = ["press_force", "seat_depth"]
PRESS_OP = {"bore_fit_um": 20, "bearing_od_mm": 12}
REQUIRED_FRACTION = 0.8   # mirrors press_check: seat needs >= 0.8 * press_force


def main() -> int:
    store = CalibrationStore.load(ROOT / "calibration" / "store.json")
    stamp = staleness(store, PRESS_PARAMS, PRESS_OP)

    pf = store.get("press_force")
    band_low = pf.band(2.0)[0]                       # press force at -2 sigma
    required = REQUIRED_FRACTION * pf.value

    problems = []
    if stamp["verdict"] != "FRESH":
        problems.append(f"press params not FRESH: {stamp['verdict']} "
                        f"(age {stamp['age_builds']} builds, "
                        f"envelope_distance {stamp['envelope_distance']}, worst {stamp['worst']})")
    if band_low < required:
        problems.append(f"no margin: press force -2sigma = {band_low:.0f} N "
                        f"< required {required:.0f} N")

    print(f"press calibration: {stamp['verdict']} | age {stamp['age_builds']} builds | "
          f"envelope_dist {stamp['envelope_distance']} | "
          f"press force {pf.value:.0f}±{2*pf.sigma:.0f} N (-2sigma {band_low:.0f} >= {required:.0f} req)")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: press sim runs on FRESH, in-envelope parameters with margin")
    return 0


if __name__ == "__main__":
    sys.exit(main())
