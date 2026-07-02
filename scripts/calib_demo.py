#!/usr/bin/env python3
"""Walk the whole calibration round-trip, printed. No sim, no writes to the store.

    python scripts/calib_demo.py

Shows the three verdicts a sim result can carry, then closes loop 3: a physical
build measures the real press force, ingest turns the surprise into a reviewable
diff (the 'pull request' against what the sim believes), apply re-anchors it, and
the staleness stamp goes green again — one build newer.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from calibration import CalibrationStore, Measurement, apply, ingest, staleness  # noqa: E402

STORE = ROOT / "calibration" / "store.json"
PRESS = ["press_force", "seat_depth"]


def show(label, stamp):
    print(f"  {label:34} {stamp['verdict']:14} "
          f"age={stamp['age_builds']}b  envelope_dist={stamp['envelope_distance']}"
          + (f"  <- {stamp['worst']}" if stamp["worst"] else ""))


def main() -> int:
    store = CalibrationStore.load(STORE)
    print(f"calibration store @ build {store.current_build}  "
          f"({len(store.params)} parameters)\n")

    # ---- 1. the staleness stamp: same query, three ways to be untrustworthy ----
    print("1. STALENESS STAMP — what a sim result carries")
    show("press @ its calibration point", staleness(store, PRESS, {"bore_fit_um": 20, "bearing_od_mm": 12}))

    store.current_build += 5      # in-memory only: pretend 5 design iterations passed
    show("press, 5 builds later (no re-anchor)", staleness(store, PRESS, {"bore_fit_um": 20, "bearing_od_mm": 12}))
    store.current_build -= 5      # undo

    show("bend springback on 3.0mm wire", staleness(store, ["bend_springback_deg"], {"wire_dia_mm": 3.0, "bend_deg": 90}))
    print("   -> FRESH means 'trust it'; STALE = re-anchor before trusting; "
          "EXTRAPOLATING = outside where reality confirmed the model\n")

    # ---- 2. loop 3: a physical build measures reality; ingest -> diff -> apply ----
    print("2. WRITEBACK — a physical build re-anchors the model")
    build = store.current_build + 1
    meas = Measurement(param="press_force", measured=322.0,
                       op_point={"bore_fit_um": 22, "bearing_od_mm": 12},
                       build=build, source="build-%d loadcell" % build)
    print(f"   physical build {build}: pressed a real bearing, load cell read "
          f"{meas.measured:.0f} N (sim assumed {store.value('press_force'):.0f} N)")

    diff = ingest(store, meas)                    # the pull request (not yet committed)
    print(f"   PROPOSED DIFF (review before commit):\n      {diff}")

    apply(store, diff, op_point=meas.op_point)    # commit + advance the clock
    print(f"   committed -> store now @ build {store.current_build}, "
          f"press_force re-anchored")
    show("press, freshly re-anchored", staleness(store, PRESS, {"bore_fit_um": 22, "bearing_od_mm": 12}))

    print("\n   (demo only — store.json on disk is untouched; a real run would "
          "review the diff and `store.save()` to commit it to git)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
