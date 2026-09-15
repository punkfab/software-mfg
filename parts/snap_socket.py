"""snap_socket.py — the SOCKET half of the snap-fit assembly (the part the peg is inserted into).

A block with a bored hole, a mouth CHAMFER (the capture cone = the swept affordance), an internal
GROOVE the peg bead snaps into, and N relief SLOTS so the mouth petals flex to admit the bead. All cuts
are coaxial from one block -> watertight single body. Mating dims come from _snapfit (shared with peg).

    py/bin/python parts/snap_socket.py -> build/snap_socket.stl
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from build123d import (Align, Box, Cone, Cylinder, Pos, Rot, export_stl)
import _snapfit as S


def part():
    block = Box(S.BLOCK, S.BLOCK, S.SOCK_H, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # through bore (overshoot both faces)
    block -= Pos(0, 0, -1) * Cylinder(S.BORE_D / 2, S.SOCK_H + 2,
                                      align=(Align.CENTER, Align.CENTER, Align.MIN))
    # mouth chamfer: a cone wide at the top face, narrowing to the bore at CHAMFER_DEPTH down (45deg)
    r_mouth = S.BORE_D / 2 + S.CHAMFER_DEPTH
    block -= Pos(0, 0, S.SOCK_H - S.CHAMFER_DEPTH) * Cone(
        S.BORE_D / 2, r_mouth, S.CHAMFER_DEPTH, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # internal detent groove the bead snaps into
    block -= Pos(0, 0, S.GROOVE_Z - S.GROOVE_W / 2) * Cylinder(
        S.GROOVE_D / 2, S.GROOVE_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # relief slots: radial slits from the bore out into the wall, top down past the groove -> flexy petals
    slot_h = (S.SOCK_H - S.GROOVE_Z) + S.GROOVE_W + 2
    for k in range(S.N_SLOTS):
        ang = 360.0 * k / S.N_SLOTS
        slot = Pos(S.BORE_D / 2 + S.SLOT_REACH / 2 - 0.5, 0, S.SOCK_H - slot_h / 2 + 1) * Box(
            S.SLOT_REACH, S.SLOT_W, slot_h)
        block -= Rot(0, 0, ang) * slot
    return block


if __name__ == "__main__":
    os.makedirs("build", exist_ok=True)
    p = part()
    export_stl(p, "build/snap_socket.stl")
    import trimesh
    m = trimesh.load("build/snap_socket.stl")
    print("snap_socket:", (m.bounds[1] - m.bounds[0]).round(1),
          "bodies:", len(m.split(only_watertight=False)), "watertight:", m.is_watertight)
