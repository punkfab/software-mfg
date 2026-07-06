#!/usr/bin/env python3
"""Gate: the hot-glue tool mounts the switcher AND its forward model is sound.

Verifies programmatically that the glue tool's coupling interface is identical to
the changer's (so the switcher can grab it, same as the shear), and that the glue
forward model (heat-ready, positive-displacement extrusion, bead timing) is coherent.
Calibration is unanchored, so results are a PREDICTION — reported, not failed.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "parts"))
sys.path.insert(0, str(ROOT / "sim"))
from calibration import CalibrationStore, staleness  # noqa: E402
import _coupling as ci  # the changer's coupling spec  # noqa: E402
import _glue as gl      # the glue tool's coupling spec  # noqa: E402
import glue_cell  # noqa: E402

GLUE_PARAMS = ["glue_melt_temp_c", "glue_heatup_tau_s", "glue_set_time_s"]


def main() -> int:
    problems = []

    # 1. the tool mounts the SAME coupling the changer expects (switchability)
    for attr in ("BORE_D", "BOLT_R", "MOUNT_ANGLES"):
        if getattr(gl, attr) != getattr(ci, attr):
            problems.append(f"coupling mismatch: {attr} {getattr(gl, attr)} != changer {getattr(ci, attr)}")

    # 2. forward model coherence
    ratio = glue_cell.extrude_ratio()
    if ratio <= 1.0:
        problems.append(f"extrude ratio must be >1 (bead longer than feed), got {ratio:.1f}")
    r = glue_cell.apply_bead("perimeter")
    if not (r["ready_s"] > 0 and r["apply_s"] > 0):
        problems.append("non-positive heat/apply time")
    if r["hold_window_s"] <= r["set_s"]:
        problems.append("hold window must exceed set time (apply + set)")

    stamp = staleness(CalibrationStore.load(ROOT / "calibration" / "store.json"),
                      GLUE_PARAMS, {"ambient_c": 20})
    trust = "VALIDATED" if stamp["verdict"] == "FRESH" else "PREDICTION (uncalibrated)"

    print(f"glue tool: coupling OD{gl.COUPLING_OD:.0f} bore{gl.BORE_D:.0f} bolts@R{gl.BOLT_R:.0f}"
          f"{gl.MOUNT_ANGLES} == changer -> switchable | "
          f"{gl.STICK_D:.1f}mm stick, {ratio:.1f}mm bead/mm feed")
    print(f"  {r['program']}: bead {r['bead_length_mm']}mm, ready {r['ready_s']}s, "
          f"apply {r['apply_s']}s, hold {r['hold_window_s']}s")
    print(f"calibration: {stamp['verdict']} -> {trust}")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: glue tool is switcher-compatible + forward model sound (a PREDICTION until measured)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
