# parts/

Parametric, code-defined geometry (build123d). One part per file.

**Convention:** each `*.py` (not `_`-prefixed) exposes a module-level `part`
holding a single build123d solid. `scripts/check_parts.py` validates it
(single watertight solid, positive volume) and exports STEP + STL.

Planned contents: `bend_disc.py` variant family (to migrate), tool-changer
kinematic coupling (arm-side + tool-side), shear tool body, datum/fiducial plate.
