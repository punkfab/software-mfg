"""station.py — the material-handling substrate: mobile stations on a shared floor.

A **station** is a generic mobile carrier, not just an arm base. Its payload is anything —
a workpiece, a parts bin, a 3D printer, a tool magazine, or a robot arm. Stations move;
materials are staged and rolled into a work envelope; a busy envelope is cleared by moving
a station out. This module is the floor-level planner for all of that.

The unifying idea: **a station is a solid on the floor, so moving one is a swept-solid
interference problem** — the same check a tool sweeping a toolpath uses (`interference.py`).
Footprints become boxes; a base move is a sweep of the moving box against the others; a
"can this material reach the work?" question is a reach test against a station's envelope.

Two things go STALE here, both borrowing the calibration/tracking staleness discipline:
a **base pose** drifts as a station moves un-observed (odometry) until a floor fiducial
re-anchors it, and a **work envelope** is only trustworthy while the arm feeding it is
calibrated. Floor frame: metres + yaw (rad); interference works in mm, converted here.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

import interference as ix

_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
from tracking.world import EXTRAPOLATING, FRESH, SIGMA_MAX_MM, STALE  # noqa: E402

BASE_DRIFT_MM_PER_M = 6.0    # odometry uncertainty added per metre driven un-observed
FIDUCIAL_SIGMA_MM = 1.5      # a floor-fiducial fix pins the base pose to ~this
FRESH_MOVES = 1              # moves-since-fix still considered fresh
DOCK_GAP_M = 0.05            # footprints within this are "docked" (adjacent, not touching)
_H = 0.15                    # nominal station height for the footprint solid (m)


def _yaw_xform_mm(x_m, y_m, yaw_rad, z_m=0.0):
    """4x4 transform (mm) placing a floor-frame pose (metres + yaw about z)."""
    T = trimesh.transformations.rotation_matrix(yaw_rad, [0, 0, 1])
    T[:3, 3] = [x_m * 1000.0, y_m * 1000.0, z_m * 1000.0]
    return T


@dataclass
class Station:
    name: str
    kind: str                       # arm | bin | printer | fixture | buffer | tool
    pose: tuple                     # (x, y, yaw) in the floor frame (m, m, rad)
    footprint: tuple = (0.3, 0.3)   # (width_x, depth_y) in metres
    mobile: bool = True
    reach_m: float = 0.0            # work-envelope radius (arms/printers that do work)
    payload: str | None = None      # material/part id currently carried
    sigma_mm: float = FIDUCIAL_SIGMA_MM   # base-pose uncertainty
    moves_since_fix: int = 0

    def footprint_mesh(self):
        """The station's footprint as a solid box centred on its own origin (mm)."""
        w, d = self.footprint
        return trimesh.creation.box(extents=(w * 1000.0, d * 1000.0, _H * 1000.0))

    def xform_mm(self):
        return _yaw_xform_mm(self.pose[0], self.pose[1], self.pose[2], _H / 2.0)

    @property
    def xy(self):
        return np.array(self.pose[:2], float)


class Floor:
    """The shop floor: stations in a shared frame, with collision-aware moves + a work-
    envelope reach test. Base-pose staleness mirrors the tracking world model."""

    def __init__(self):
        self.stations: dict[str, Station] = {}

    def add(self, station: Station):
        self.stations[station.name] = station
        return self

    # --- geometry: stations are solids, reuse the interference scene ---
    def scene(self, exclude=()):
        sc = ix.Scene()
        for n, s in self.stations.items():
            if n in exclude:
                continue
            sc.place(n, s.footprint_mesh(), s.xform_mm())
        return sc

    def clashes(self, clearance_m=0.0):
        """Any stations whose footprints overlap (a layout error / a blocked dock)."""
        return self.scene().interferences()

    def clearances(self, near_m=0.3):
        """Near-miss footprint gaps under near_m (tight aisles)."""
        return self.scene().clearances(near_mm=near_m * 1000.0)

    # --- motion: a base move is a swept-solid interference check ---
    def can_move(self, name, to_pose, steps=12):
        """Would driving `name` from its pose to `to_pose` foul any other station?
        Sweeps the moving footprint against the static ones. Returns the sweep result."""
        s = self.stations[name]
        statics = self.scene(exclude=(name,))
        x0, y0, a0 = s.pose
        x1, y1, a1 = to_pose
        xforms = []
        for t in np.linspace(0.0, 1.0, steps):
            xforms.append(_yaw_xform_mm(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t,
                                        a0 + (a1 - a0) * t, _H / 2.0))
        return ix.sweep(s.footprint_mesh(), xforms, statics)

    def move(self, name, to_pose, steps=12):
        """Drive a station to a new base pose if the lane is clear. Commits the move and
        grows base-pose uncertainty (odometry drift); refuses on a foul. Returns a dict."""
        s = self.stations[name]
        if not s.mobile:
            return {"ok": False, "reason": "station is fixed"}
        sweep = self.can_move(name, to_pose, steps)
        if not sweep["ok"]:
            return {"ok": False, "reason": "would foul", "blocked_by": sweep["first_hit"]}
        dist_m = float(np.linalg.norm(np.array(to_pose[:2]) - s.xy))
        s.pose = tuple(to_pose)
        s.sigma_mm += BASE_DRIFT_MM_PER_M * dist_m
        s.moves_since_fix += 1
        return {"ok": True, "dist_m": round(dist_m, 3), "staleness": self.staleness(name)}

    def route(self, name, waypoints, steps=12):
        """Drive through a sequence of base poses, sweeping each leg for clearance. Stops
        at the first blocked leg (the point of a route: steer a carrier around obstacles)."""
        legs = []
        for wp in waypoints:
            r = self.move(name, wp, steps)
            legs.append(r)
            if not r["ok"]:
                return {"ok": False, "legs": legs, "blocked_at": tuple(wp)}
        return {"ok": True, "legs": legs, "final": self.stations[name].pose}

    def fix(self, name, observed_pose=None):
        """Re-anchor a base pose from a floor fiducial. Resets uncertainty + the clock."""
        s = self.stations[name]
        if observed_pose is not None:
            s.pose = tuple(observed_pose)
        s.sigma_mm = FIDUCIAL_SIGMA_MM
        s.moves_since_fix = 0
        return self.staleness(name)

    def staleness(self, name):
        s = self.stations[name]
        if s.sigma_mm > SIGMA_MAX_MM:
            verdict = EXTRAPOLATING
        elif s.moves_since_fix > FRESH_MOVES:
            verdict = STALE
        else:
            verdict = FRESH
        return {"verdict": verdict, "moves_since_fix": s.moves_since_fix,
                "sigma_mm": round(s.sigma_mm, 2)}

    # --- work envelope: can a station's payload be worked by an arm? ---
    def docked(self, a, b, gap_m=DOCK_GAP_M):
        """Are two stations adjacent (footprints within gap_m of touching)?"""
        A, B = self.stations[a], self.stations[b]
        d = float(np.linalg.norm(A.xy - B.xy))
        touch = (max(A.footprint) + max(B.footprint)) / 2.0   # half-footprint each
        return d <= touch + gap_m

    def reachable(self, arm_name, target_name):
        """Is target_name's payload inside arm_name's work envelope (planar reach)?"""
        arm, tgt = self.stations[arm_name], self.stations[target_name]
        d = float(np.linalg.norm(arm.xy - tgt.xy))
        return {"reachable": 0.02 <= d <= arm.reach_m, "distance_m": round(d, 3),
                "reach_m": arm.reach_m}

    def stage_into_envelope(self, carrier_name, arm_name, dock_pose, via=(), steps=12):
        """The staging flow: route a material carrier (optionally around obstacles via a
        list of waypoints) to a dock pose beside an arm, then confirm its payload lands in
        the arm's work envelope. One call = route + reach — the whole material-in step."""
        rt = self.route(carrier_name, list(via) + [dock_pose], steps)
        if not rt["ok"]:
            return {"ok": False, "stage": "move", **rt}
        reach = self.reachable(arm_name, carrier_name)
        return {"ok": reach["reachable"], "stage": "reach", "route": rt, "reach": reach,
                "payload": self.stations[carrier_name].payload}
