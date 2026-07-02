"""store.py — the calibrated parameter vector: the real source of truth.

A hardware CI loop TESTS a simulation but SHIPS a physical part. The sim is only
trustworthy where its parameters have been anchored to reality. This module holds
those parameters as one git-tracked file. CAD, sim, and machine config all read
from here; calibration (loop 3) writes back here.

A calibrated parameter is not a number — it is a number PLUS how far you can
trust it:
  - value / sigma      distributional, not exact (physical parts scatter)
  - op_point           the conditions it was measured at
  - envelope           the range of conditions over which the value is believed
  - anchored_at_build  which physical build last confirmed it (the staleness clock)

`current_build` is that clock: a monotone counter bumped by each physical build.
Staleness is measured in builds-since-anchor, NOT wall-clock — the target is
moving, and what matters is how many design iterations you have run since reality
last spoke. A green sim on stale calibration is a cache hit on stale data.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

FRESH_AGE = 3   # builds-since-anchor still considered fresh
FRESH, STALE, EXTRAPOLATING = "FRESH", "STALE", "EXTRAPOLATING"


@dataclass
class Parameter:
    value: float
    unit: str = ""
    sigma: float = 0.0                              # 1-sigma uncertainty (distributional)
    op_point: dict = field(default_factory=dict)    # conditions at calibration
    envelope: dict = field(default_factory=dict)    # {dim: [lo, hi]} trusted range
    anchored_at_build: int = 0
    provenance: str = ""
    residual: float = 0.0                           # last (measured - predicted)

    def band(self, k: float = 2.0):
        """The [value-k*sigma, value, value+k*sigma] set — 'green means
        in-tolerance-WITH-MARGIN' checks against the low/high edge, not nominal."""
        return [self.value - k * self.sigma, self.value, self.value + k * self.sigma]


@dataclass
class CalibrationStore:
    path: Path
    current_build: int
    params: dict                                    # name -> Parameter

    @classmethod
    def load(cls, path) -> "CalibrationStore":
        path = Path(path)
        raw = json.loads(path.read_text())
        params = {k: Parameter(**v) for k, v in raw["params"].items()}
        return cls(path=path, current_build=raw.get("current_build", 0), params=params)

    def save(self) -> None:
        out = {
            "current_build": self.current_build,
            "params": {k: asdict(v) for k, v in self.params.items()},
        }
        self.path.write_text(json.dumps(out, indent=2) + "\n")

    def get(self, name: str) -> Parameter:
        if name not in self.params:
            raise KeyError(f"no calibrated parameter {name!r}")
        return self.params[name]

    def value(self, name: str, default=None):
        """What the sim/CAD consume. Returns `default` if uncalibrated (so a sim
        still runs before its parameters have been anchored — it just isn't
        trustworthy, which is exactly what the staleness stamp will say)."""
        p = self.params.get(name)
        return p.value if p is not None else default


def _envelope_distance(param: Parameter, op_point: dict):
    """Fraction of the envelope width the op_point sits OUTSIDE the trusted range.
    0.0 = inside (interpolating); >0 = extrapolating. Worst (max) over declared
    dims. A dim not present in op_point is not checked."""
    worst, worst_dim = 0.0, None
    for dim, rng in param.envelope.items():
        if dim not in op_point:
            continue
        lo, hi = rng
        x = op_point[dim]
        width = (hi - lo) or 1.0
        d = max(0.0, lo - x, x - hi) / width
        if d > worst:
            worst, worst_dim = d, dim
    return worst, worst_dim


def staleness(store: CalibrationStore, param_names, op_point=None) -> dict:
    """The stamp a sim result carries. Given the parameters a result depended on
    and the operating point it ran at, how much do we still trust it?

      age_builds         builds since the worst parameter was last anchored
      envelope_distance  how far outside the trusted envelope we extrapolated
      verdict            FRESH | STALE | EXTRAPOLATING
      worst              the parameter (and dim) that drove the verdict
    """
    op_point = op_point or {}
    age, dist, worst = 0, 0.0, None
    for name in param_names:
        p = store.get(name)
        a = store.current_build - p.anchored_at_build
        if a > age:
            age = a
        d, dim = _envelope_distance(p, op_point)
        if d > dist:
            dist, worst = d, f"{name}.{dim}"
    if dist > 0:
        verdict = EXTRAPOLATING
    elif age > FRESH_AGE:
        verdict = STALE
    else:
        verdict = FRESH
    return {"age_builds": age, "envelope_distance": round(dist, 3),
            "verdict": verdict, "worst": worst}
