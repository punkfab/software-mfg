"""measure.py — ingest a physical measurement as a PULL REQUEST against the store.

This is loop 3: reality -> parameter update. A physical build is a MEASUREMENT; it
does not silently overwrite the model. `ingest` compares what the part actually did
to what the model predicted and returns a ParamDiff (old -> new, the residual the
sim got wrong, updated uncertainty) for review. `apply` commits it and advances the
build clock.

Same discipline as featuretree's IR round-trip: the reverse leg produces a
reviewable diff, not a mutation. You read the diff, decide it's real (not a bad
measurement), then commit it — a git-tracked change to what the sim believes.
"""

from __future__ import annotations

from dataclasses import dataclass

from .store import CalibrationStore, Parameter


@dataclass
class Measurement:
    param: str
    measured: float          # what the physical part actually did
    op_point: dict           # the conditions of THIS build
    build: int               # the physical-build id this came from
    source: str = ""         # instrument / coupon / log reference (the 'disassembler')


@dataclass
class ParamDiff:
    param: str
    old_value: float
    new_value: float
    residual: float          # measured - model-predicted: what the sim got wrong
    old_sigma: float
    new_sigma: float
    build: int
    source: str

    def model_error_pct(self) -> float:
        return 100.0 * self.residual / (self.old_value or 1.0)

    def __str__(self) -> str:
        return (f"{self.param}: {self.old_value:.4g} -> {self.new_value:.4g} "
                f"(residual {self.residual:+.4g}, {self.model_error_pct():+.1f}% model error) "
                f"| sigma {self.old_sigma:.3g} -> {self.new_sigma:.3g}  @build {self.build}")


def ingest(store: CalibrationStore, m: Measurement, sigma_smooth: float = 0.5) -> ParamDiff:
    """Compare a measurement to the model's current belief; propose an update.

    The residual (measured - predicted) is the model error at this operating point:
    the number a pure sim-CI loop can never see. Uncertainty tracks the residual
    magnitude — recent agreement earns tighter sigma; a surprise widens it.
    Returns the diff WITHOUT applying it (the 'pull request')."""
    p = store.get(m.param)
    residual = m.measured - p.value
    new_sigma = (1 - sigma_smooth) * p.sigma + sigma_smooth * abs(residual)
    return ParamDiff(m.param, p.value, m.measured, residual,
                     p.sigma, new_sigma, m.build, m.source)


def apply(store: CalibrationStore, diff: ParamDiff, op_point=None,
          envelope_grow: bool = True) -> Parameter:
    """Commit a ParamDiff: new value + uncertainty, re-anchor to this build, and
    (optionally) grow the trusted envelope to include the point we just measured.
    Advances the build clock — every later sim is now measured against this."""
    p = store.get(diff.param)
    p.value = diff.new_value
    p.sigma = diff.new_sigma
    p.residual = diff.residual
    p.anchored_at_build = diff.build
    p.provenance = diff.source
    if op_point:
        p.op_point = dict(op_point)
        if envelope_grow:
            for dim, x in op_point.items():
                lo, hi = p.envelope.get(dim, [x, x])
                p.envelope[dim] = [min(lo, x), max(hi, x)]
    store.current_build = max(store.current_build, diff.build)
    return p
