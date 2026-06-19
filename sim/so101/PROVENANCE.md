# SO-101 model — provenance & attribution

Vendored (unmodified) snapshot of the SO-101 simulation assets.

- **Upstream:** https://github.com/TheRobotStudio/SO-ARM100 — `Simulation/SO101/`
- **Pinned commit:** `aec17bbc256d1a7342d53aaa4950595d4c30b40d`
- **License:** Apache-2.0 (see `UPSTREAM_LICENSE`)
- **Retrieved:** 2026-06-18
- **Model generated** by TheRobotStudio with onshape-to-robot; STS3215 servo
  properties adapted from the Open Duck Mini project.

## Contents (verbatim from upstream)

- `scene.xml` — top-level scene; `<include>`s the arm + floor/lighting
- `so101_new_calib.xml` — arm model, **new** calibration (joint zero = mid-range) — default
- `so101_old_calib.xml` — arm model, **old** calibration (joint zero = horizontal extension)
- `joints_properties.xml`, `so101_new_calib.urdf` — aux / ROS reference
- `assets/*.stl` — 13 collision/visual meshes (`meshdir="assets"`)

## Why vendored (not composed by reference)

Unlike `../wirebender` (a live sibling repo we compose by reference), SO-101 is a
third-party upstream model. We pin a reproducible snapshot so the sim is stable
and offline-buildable. **Do not edit these files** — software-mfg additions
(workcell table, datums, tool changer) compose the model from the outside
(`<include>` or the MjSpec API), keeping this snapshot a clean, updatable mirror.

## Verified

`python scripts/so101_check.py` — compiles, 6 DOF + 6 position actuators, position
control tracks targets, end effector moves under FK.

## Updating the snapshot

Re-pin a newer commit, re-download `Simulation/SO101/` (xmls + `assets/*.stl`) and
the root `LICENSE`, update the commit SHA above, and re-run `so101_check.py`.
