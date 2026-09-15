"""snap_peg.py — the PEG half of the snap-fit assembly (the part the arm picks and inserts).

A gripper-holdable base block with an axisymmetric peg on top: tapered lead-in nose (finds the socket
chamfer) + annular snap bead (snaps into the socket groove). The peg is one revolved solid welded to
the block, so it stays watertight/single-body. Mating dims come from _snapfit (shared with the socket).

    py/bin/python parts/snap_peg.py -> build/snap_peg.stl
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build123d import (Align, Axis, BuildLine, BuildPart, BuildSketch, Box, Plane, Polyline, Pos,
                       export_stl, make_face, revolve)
import _snapfit as S


def part():
    # base plate the gripper holds and that presents the peg at a known pose
    base = Box(S.BLOCK, S.BLOCK, S.BASE_T, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # the peg: revolve the shared half-profile about Z -> one clean solid, then weld onto the plate
    with BuildPart() as peg:
        with BuildSketch(Plane.XZ):
            with BuildLine():
                Polyline(S.peg_profile(), close=True)
            make_face()
        revolve(axis=Axis.Z)
    return base + Pos(0, 0, S.BASE_T) * peg.part      # profile dips to z=-1 -> overlaps into the plate


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = part()
    export_stl(p, "build/snap_peg.stl")
    import trimesh
    m = trimesh.load("build/snap_peg.stl")
    print("snap_peg:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
