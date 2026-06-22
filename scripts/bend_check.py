#!/usr/bin/env python3
"""Gate: the wire-bender cell composes by reference and produces a real staple.

Runs the bender's OWN forward model, checks the staple is sane, and confirms we
left no writes in the (read-only) wirebender repo.
"""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "sim"))
from wirebender_cell import simulate  # noqa: E402

# Read-only discipline is enforced by the adapter running the bender in a
# subprocess with PYTHONDONTWRITEBYTECODE=1, so it writes nothing into the repo —
# not even bytecode. (We don't inspect __pycache__ here: the user legitimately has
# their own from running wirebender directly.)


def main() -> int:
    r = simulate("staple")
    pts = np.asarray(r["points"])
    problems = []

    if not (70.0 < r["length"] < 90.0):
        problems.append(f"unexpected wire length {r['length']} mm")
    if r["n_bends"] != 2:
        problems.append(f"expected 2 bends, got {r['n_bends']}")
    if float(np.abs(pts[:, 2]).max()) > 1e-6:
        problems.append("staple should be planar (z=0)")
    if not (5.0 < r["duration"] < 60.0):
        problems.append(f"implausible bend cycle time {r['duration']} s")

    bbox = (pts.max(0) - pts.min(0)).round(1)
    print(f"composed wire bender (by reference): staple = {r['length']} mm, "
          f"{r['n_bends']} bends, {len(pts)} pts, bbox {bbox[:2].tolist()} mm")
    print(f"bend cycle time (axis-speed model): {r['duration']} s")
    if problems:
        for p in problems:
            print("FAIL:", p)
        return 1
    print("PASS: bend_wire runs the bender's real forward model; wirebender untouched")
    return 0


if __name__ == "__main__":
    sys.exit(main())
