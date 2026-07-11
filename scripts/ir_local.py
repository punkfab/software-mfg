"""ir_local.py — software-mfg's project-specific feature-IR samples.

featuretree's `ir` (the neutral CAD DSL + emitters) is composed BY REFERENCE from the
../featuretree sibling repo — this module does NOT fork it. It imports the library and
adds the one sample that belongs to THIS project: `coupling_plate`, the real tool-changer
coupling blank (cf. parts/_coupling.py). Everything else — the DSL builders (sketch/pad/
pocket/fillet/part), update_from_freecad, and the library's own SAMPLES — comes straight
from the sibling, so upstream changes flow through automatically.

  from ir_local import IR, SAMPLES     # IR = the sibling library; SAMPLES = lib + local
"""

import math
import sys

from freecad_cmd import featuretree_root

sys.path.insert(0, str(featuretree_root()))
import ir as IR  # noqa: E402  (the featuretree library, composed by reference)

# re-export the library's round-trip helper so callers need one import
update_from_freecad = IR.update_from_freecad


def coupling_plate():
    """The real tool-changer coupling blank (cf. parts/_coupling.py): a Ø50x6 disc with a
    Ø12 central bore + 3 M3 mounting holes, a filleted top rim, and a bore counterbore.

    The fillet selects the disc's top outer edge by QUERY (not a stored kernel edge id) and
    the counterbore sketch is attached to the top face by query — so both survive edits and
    rebuilds. A real project part, round-tripped through FreeCAD by name.
    """
    import math
    bolts = [(round(21 * math.cos(math.radians(a)), 4),
              round(21 * math.sin(math.radians(a)), 4)) for a in (30, 150, 270)]
    return IR.part(
        "coupling_plate",
        IR.sketch("disc_outline", "XY", circles=[(0, 0, 25)]),
        IR.pad("disc", "disc_outline", length=6),
        IR.fillet("rim_round", radius=1.0, select={"circles": "top_outer"}),
        IR.sketch("holes", "XY", circles=[(0, 0, 6)] + [(x, y, 1.7) for x, y in bolts]),
        IR.pocket("drill", "holes", through=True),
        # counterbore recess around the bore, drilled from the TOP FACE (face-attached
        # sketch — the face is chosen by query, then coords map into its local frame)
        IR.sketch("recess_sk", circles=[(0, 0, 9)], on={"face_of": "drill", "side": "top"}),
        IR.pocket("recess", "recess_sk", through=False, length=1.5),
    )


def kiwi_wheel():
    """The kiwi-v10 omni wheel, measured off its STEP: a body-of-revolution disc (OD 68 / r34 for
    axial -19.5..+8, stepping to a Ø12 hub boss, ~Ø2 shaft hole) with two staggered rings of roller
    POCKETS cut into the rim — 4 tangent bores per ring, ring 2 rotated half a pitch (45deg), at the
    measured pin pitch (mount_r 30). This is the swept form PLUS the roller cutouts, as an editable
    feature tree (Sketch -> Revolution -> two PolarPocket rings).

    Fidelity notes: the pockets are placed at z=-12/-1 (inside the disc) rather than the exact pin
    rows (-15/+4) — those straddle the hub-boss step and fragment the solid; nudging them in keeps a
    single watertight solid. Still the solid shell (~88k mm^3), not the fully-hollow real wheel
    (~43k). OD 68 / width 39 / 8 staggered roller pockets match.
    """
    # Revolve profile (radius, axial) with a servo-side STANDOFF BOSS: the Ø22 boss (z=-27.5..-19.5)
    # sticks out toward the servo so the roller disc (z=-19.5..+8) clears the STS3215 body; the horn
    # seats in the boss end. Disc OD 68. (Servo-drive variant: the original outboard Ø12 hub is
    # replaced by a back-access bore.)
    profile = [(1.1, -27.5), (11.0, -27.5), (11.0, -19.5), (34.0, -19.5), (34.0, 8.0), (1.1, 8.0)]
    # STS3215 horn bolt circle (ST-3215-C047): Ø14 BCD, 4× M3 clearance (Ø3.4).
    bcr = 7.0
    bolts = [(round(bcr * math.cos(math.radians(a)), 4), round(bcr * math.sin(math.radians(a)), 4))
             for a in (45, 135, 225, 315)]
    return IR.part(
        "kiwi_wheel",
        IR.sketch("section", "XZ", polys=[profile]),
        IR.revolve("body", "section", angle=360.0),
        IR.polar_pocket("rollers_a", radius=6.0, length=16.0, mount_r=30.0, z=-12.0, count=4, phase=0.0),
        IR.polar_pocket("rollers_b", radius=6.0, length=16.0, mount_r=30.0, z=-1.0, count=4, phase=45.0),
        # --- servo-horn mount on the boss end (bottom, z=-27.5) ---
        IR.sketch("horn_recess_sk", circles=[(0, 0, 10.25)], on={"face_of": "body", "side": "bottom"}),
        IR.pocket("horn_recess", "horn_recess_sk", through=False, length=3.0),   # Ø20.5 horn seat
        IR.sketch("hub_bore_sk", circles=[(0, 0, 4.6)], on={"face_of": "body", "side": "bottom"}),
        IR.pocket("hub_bore", "hub_bore_sk", through=False, length=6.0),         # Ø9.2 clears the Ø9 hub
        # back-access counterbore (from the disc back, z=+8): drop M3 screws in here so they only span
        # the boss to the horn — not the whole wheel
        IR.sketch("access_sk", circles=[(0, 0, 9.0)], on={"face_of": "body", "side": "top"}),
        IR.pocket("access", "access_sk", through=False, length=24.0),            # Ø18 down to z=-16
        IR.sketch("horn_bolts_sk", circles=[(x, y, 1.7) for x, y in bolts],
                  on={"face_of": "body", "side": "bottom"}),
        IR.pocket("horn_bolts", "horn_bolts_sk", through=True),                  # 4× M3, boss->access bore
    )


# the library's samples + this project's own, in one dict
SAMPLES = {**IR.SAMPLES, "coupling_plate": coupling_plate, "kiwi_wheel": kiwi_wheel}
