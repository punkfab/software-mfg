"""calibration — the round-trip's reality leg.

The sim is a CACHE of reality; this package is the invalidation signal. It holds the
calibrated parameter vector (the real source of truth), ingests physical
measurements as reviewable diffs, and stamps every sim result with how far it can
still be trusted (builds-since-anchor + how far it extrapolated).

    from calibration import CalibrationStore, staleness, Measurement, ingest, apply
"""

from .measure import Measurement, ParamDiff, apply, ingest
from .store import (
    EXTRAPOLATING,
    FRESH,
    STALE,
    CalibrationStore,
    Parameter,
    staleness,
)

__all__ = [
    "CalibrationStore", "Parameter", "staleness", "FRESH", "STALE", "EXTRAPOLATING",
    "Measurement", "ParamDiff", "ingest", "apply",
]
