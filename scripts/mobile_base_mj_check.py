#!/usr/bin/env python3
"""Gate: the MuJoCo dynamic base model corroborates the analytic statics.

Two independent models of the same physics should agree — the design ↔ sim round-trip
applied to the base. This gate asserts:
  - at rest, every design settles stable on its contacts (no phantom tilt);
  - at the real printer-pick reach, the base stays steady (the operating result);
  - over-extended, the base TIPS where the moment balance predicts (sim onset ≈ analytic,
    within the onset-detection lag) — the corroboration;
  - tip reach grows with footprint (diff2 < mecanum4 < omni_diff < footed4) — ordering holds.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim"))
import mobile_base as mb  # noqa: E402
import mobile_base_mj as mj  # noqa: E402

ONSET_TOL_M = 0.2   # sim onset vs analytic (onset detection lags CG-crossing by ~0.1m)


def main() -> int:
    problems = []
    reach = {}
    for k, d in mb.DESIGNS.items():
        r = mj.run_reach(d, mb.PRINTER_PICK, ramp_s=8.0, reach_range=1.4)
        reach[k] = r

        # 1. settles stable at rest
        if r["settle_tilt_deg"] > 2.0:
            problems.append(f"{k}: unstable at rest ({r['settle_tilt_deg']}°)")
        # 2. eventually tips when over-extended, and onset ~ prediction
        if not r["tips"]:
            problems.append(f"{k}: never tipped even over-extended to 1.4m")
        else:
            err = abs(r["tip_reach_m"] - r["predict_tip_reach_m"])
            if err > ONSET_TOL_M:
                problems.append(f"{k}: sim tip {r['tip_reach_m']}m vs predict "
                                f"{r['predict_tip_reach_m']}m — off by {err:.2f}m (>{ONSET_TOL_M})")
            if r["tip_reach_m"] < r["predict_tip_reach_m"] - 0.05:
                problems.append(f"{k}: sim tipped BEFORE prediction — model unsafe")

    # 3. operating: steady at the real working reach (15cm extra forward slide)
    for k, d in mb.DESIGNS.items():
        s = mj.reach_stable(d, mb.PRINTER_PICK, 0.15)
        if s["tilt_deg"] > 5.0:
            problems.append(f"{k}: not steady at the working reach ({s['tilt_deg']}°)")

    # 4. tip reach ordering follows footprint
    order = ["diff2", "mecanum4", "omni_diff", "footed4"]
    tips = [reach[k]["tip_reach_m"] for k in order]
    if any(a is not None and b is not None and a >= b for a, b in zip(tips, tips[1:])):
        problems.append(f"tip-reach ordering not monotonic in footprint: {dict(zip(order, tips))}")

    # --- report ---
    print("MuJoCo dynamic tip-over vs analytic prediction (over-extend probe):\n")
    for k in order:
        r = reach[k]
        print(f"  {r['design']:28s} sim onset {r['tip_reach_m']}m  vs predict "
              f"{r['predict_tip_reach_m']}m  (settle {r['settle_tilt_deg']}°, peak {r['max_tilt_deg']}°)")

    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("\nPASS: the dynamic sim tips where the statics predict — two models corroborate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
