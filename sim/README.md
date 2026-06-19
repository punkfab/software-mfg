# sim/

MuJoCo (MJCF) models + scenes — the fast inner iteration loop.

## so101/

Vendored snapshot of the official **SO-101** model from `TheRobotStudio/SO-ARM100`
(Apache-2.0, pinned commit — see `so101/PROVENANCE.md`). 6-DOF arm + gripper,
position actuators. **Do not edit these files**; compose additions (workcell
table, datums, tool changer) from the outside via `<include>` or the MjSpec API.

Validate / load:

```bash
python scripts/so101_check.py      # compile + position-control move + FK check
python -m mujoco.viewer --mjcf sim/so101/scene.xml   # interactive viewer (needs a display)
```

Control interface: `data.ctrl[:] = desired joint angles (rad)`, order =
`[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper]`.

Coming next (PLAN.md Phase 1–2): a `workcell` scene (arm + table + fiducial
datum) composed without touching the snapshot, then the tool-changer + shear.
The wire-bender cell already has its own MuJoCo sim in `../wirebender/sim`, which
we'll compose by reference at the integration milestone (Phase 3).
