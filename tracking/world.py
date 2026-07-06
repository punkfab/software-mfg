"""world.py — the tracked world model: where every part is, and how sure we are.

Pose is just another thing that goes STALE. A part we've grasped but not re-observed
is known only by dead-reckoning the arm's forward kinematics; its uncertainty grows
until the estimate is EXTRAPOLATING and must be re-anchored by an observation. This is
the calibration layer's FRESH / STALE / EXTRAPOLATING idea (see calibration/) applied to
6-DoF pose instead of a scalar parameter — same discipline, same vocabulary.

Camera-agnostic: observations come from `tracking.observe` (simulated from sim ground
truth now; a real fiducial / CAD-pose estimator later). The world model is identical
whichever sensor feeds it — swapping perception doesn't touch this file.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

FRESH, STALE, EXTRAPOLATING = "FRESH", "STALE", "EXTRAPOLATING"
FRESH_STEPS = 3           # observations-ago still considered fresh
SIGMA_MAX_MM = 5.0        # dead-reckoning uncertainty beyond this -> EXTRAPOLATING
DRIFT_MM = 0.8            # uncertainty added per un-observed carry step
SENSOR_SIGMA_MM = 0.5     # a fresh observation pins the pose to ~this

FREE, PLACED = "free", "placed"


@dataclass
class Pose:
    xyz: tuple                              # metres
    quat: tuple = (1.0, 0.0, 0.0, 0.0)      # wxyz

    def trans_err_mm(self, other: "Pose") -> float:
        return 1000.0 * float(np.linalg.norm(np.array(self.xyz) - np.array(other.xyz)))

    def ang_err_deg(self, other: "Pose") -> float:
        d = abs(float(np.dot(self.quat, other.quat)))
        return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, d))))


@dataclass
class TrackedPart:
    name: str
    cad_ref: str
    nominal: Pose                # where the assembly says it should end up (datum frame)
    est: Pose                    # current best estimate
    sigma_mm: float = SENSOR_SIGMA_MM
    grasp_state: str = FREE      # FREE | <arm-id> | PLACED
    last_obs_step: int = 0


class WorldModel:
    def __init__(self):
        self.parts: dict[str, TrackedPart] = {}
        self.step = 0

    def add(self, name, cad_ref, nominal, est=None):
        self.parts[name] = TrackedPart(name, cad_ref, nominal, est or nominal)

    def tick(self):
        self.step += 1

    def grasp(self, name, arm):
        self.parts[name].grasp_state = arm

    def carry(self, name, new_pose):
        """Dead-reckon a grasped part to a new pose (inferred from arm FK). Uncertainty
        GROWS — we haven't seen it move, only inferred it."""
        p = self.parts[name]
        p.est = new_pose
        p.sigma_mm += DRIFT_MM

    def observe(self, name, pose, sensor_sigma_mm=SENSOR_SIGMA_MM):
        """Re-anchor to a measurement (fiducial / CAD-pose). Resets uncertainty + the
        staleness clock. Returns the correction applied (observation vs prior estimate)."""
        p = self.parts[name]
        correction = p.est.trans_err_mm(pose)
        p.est = pose
        p.sigma_mm = sensor_sigma_mm
        p.last_obs_step = self.step
        return correction

    def place(self, name, pose=None):
        p = self.parts[name]
        p.grasp_state = PLACED
        if pose is not None:
            p.est = pose

    def staleness(self, name) -> dict:
        """The pose trust stamp — same three verdicts as a calibrated parameter."""
        p = self.parts[name]
        age = self.step - p.last_obs_step
        if p.sigma_mm > SIGMA_MAX_MM:
            verdict = EXTRAPOLATING
        elif age > FRESH_STEPS:
            verdict = STALE
        else:
            verdict = FRESH
        return {"verdict": verdict, "age": age, "sigma_mm": round(p.sigma_mm, 2)}

    def place_error(self, name) -> dict:
        """The tracked pose vs. the CAD-nominal pose (the assembly check)."""
        p = self.parts[name]
        return {"trans_mm": round(p.est.trans_err_mm(p.nominal), 2),
                "ang_deg": round(p.est.ang_err_deg(p.nominal), 2)}
