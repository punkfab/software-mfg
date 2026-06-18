"""Pipeline smoke-test part — a plate with a through hole.

Placeholder that proves the parts -> validate -> STEP/STL pipeline works end to
end. It will be removed once the real bend_disc.py variant family is migrated in
(see PLAN.md, Phase 0).

Convention: every part module exposes a module-level `part` (a build123d solid).
"""

from build123d import BuildPart, Box, Cylinder, Mode

# --- parameters (mm) ---
PLATE_L = 40.0
PLATE_W = 30.0
PLATE_T = 10.0
HOLE_D = 8.0

with BuildPart() as bp:
    Box(PLATE_L, PLATE_W, PLATE_T)
    Cylinder(radius=HOLE_D / 2, height=PLATE_T, mode=Mode.SUBTRACT)

part = bp.part
