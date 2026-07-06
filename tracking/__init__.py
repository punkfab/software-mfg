"""tracking — the world model: CAD-referenced part-pose tracking for assembly.

Where every part is, how sure we are (pose staleness, reusing calibration's
FRESH/STALE/EXTRAPOLATING), which part is grasped, and whether a placement matches the
CAD-nominal pose. Camera-agnostic: fed by simulated observations now, a real
fiducial/CAD-pose estimator later.

    from tracking import WorldModel, Pose, load_assembly, verify
"""

from .assembly import SAMPLE, load as load_assembly, verify
from .world import (
    EXTRAPOLATING,
    FRESH,
    PLACED,
    STALE,
    Pose,
    TrackedPart,
    WorldModel,
)

__all__ = [
    "WorldModel", "TrackedPart", "Pose", "FRESH", "STALE", "EXTRAPOLATING", "PLACED",
    "load_assembly", "verify", "SAMPLE",
]
