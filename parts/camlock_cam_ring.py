"""camlock_cam_ring.py — the rotating LOCK RING of the servo-driven draw-lock changer.

Sits in the arm-plate bore, axially retained by its top FLANGE (which bears on the arm
plate's top face and takes the draw reaction). Its bore is open for the tool post; 3
internal RAMPED catches (interrupted ACME-thread segments) reach in to hook the post lugs.
A servo swings the side DRIVE ARM ~ENGAGE_ANGLE; the ramps draw the post home. A shallow
lead makes it self-locking, so it holds with the servo unpowered. [_camlock §2-3]

    python parts/camlock_cam_ring.py   # via check_parts -> exports/
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build123d import Align, Box, Cylinder, Pos, Rot  # noqa: E402

import _camlock as c  # noqa: E402

C, MN = Align.CENTER, Align.MIN


def _build():
    # ring body + top flange (sink 1mm so they weld)
    part = Cylinder(c.CAM_OD / 2, c.CAM_H, align=(C, C, MN))
    part += Pos(0, 0, c.CAM_H - 1.0) * Cylinder(c.CAM_FLANGE_OD / 2, c.CAM_FLANGE_T + 1.0,
                                                align=(C, C, MN))
    # central bore (lugs pass through here at the gaps)
    part -= Pos(0, 0, -1) * Cylinder(c.GAP_BORE_R, c.CAM_H + c.CAM_FLANGE_T + 2,
                                     align=(C, C, MN))
    # 3 ramped catches projecting inward at the ring bottom (weld into the body wall)
    for a in c.CATCH_ANGLES:
        part += c.ramped_catch(c.R_CATCH_IN, c.GAP_BORE_R + 1.0, 0.0, c.CATCH_H, a,
                               c.CATCH_ARC, c.DRAW)

    # side drive arm (a servo horn / link swings this) with a link-pin hole at the tip
    arm_z = c.CAM_H - c.CAM_FLANGE_T
    part += Rot(0, 0, c.DRIVE_ARM_ANGLE) * Pos(c.CAM_FLANGE_OD / 2 - 2.0, 0, arm_z) \
        * Box(c.DRIVE_ARM_LEN + 2.0, c.DRIVE_ARM_W, c.CAM_FLANGE_T, align=(MN, C, MN))
    px = c.CAM_FLANGE_OD / 2 + c.DRIVE_ARM_LEN - 3.0
    part -= Rot(0, 0, c.DRIVE_ARM_ANGLE) * Pos(px, 0, -1) \
        * Cylinder(c.DRIVE_PIN_D / 2, c.CAM_H + 2, align=(C, C, MN))
    return part


part = _build()


if __name__ == "__main__":
    from build123d import export_step, export_stl
    os.makedirs("exports", exist_ok=True)
    export_step(part, "exports/camlock_cam_ring.step")
    export_stl(part, "exports/camlock_cam_ring.stl")
    import trimesh
    m = trimesh.load("exports/camlock_cam_ring.stl")
    print("camlock cam-ring:", [round(v, 1) for v in part.bounding_box().size],
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
