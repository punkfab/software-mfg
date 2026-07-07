# 3D scanning — the reality-capture leg

Scanning is how real geometry re-enters the loop. It isn't a new subsystem so much as the
**real sensor** behind three things the repo already models with synthetic data:

| Hook | What scanning provides | Today | Built |
|---|---|---|---|
| **Perception for tracking** | scan → ICP to CAD → 6-DoF pose → `tracking.observe()` / `Floor.fix()` | synthetic observations | ✅ `sim/scanning.py` |
| **Calibration reality-leg (geometry)** | per-point deviation vs CAD = a measured residual writeback | scalar params only | ✅ deviation map |
| **As-built verification** | scan the finished part/assembly → deviation vs CAD nominal | `assembly.verify` from tracked pose | ✅ deviation |
| **Incoming-part reverse-eng** | scan a bought part/feedstock with no CAD → mesh into the planner | — | future |
| **Scene collision geometry** | depth scan of the cell → mesh → `interference.py` vs real fixtures | CAD-only scene | future |

## Built this pass — `sim/scanning.py` (gated by `scanning_check.py`)

Self-contained (trimesh ICP + sampling + proximity; no open3d). A scan is *simulated* from
CAD ground truth — sample the surface, cull to each sensor view, add noise, place at the
part's real pose, optionally inject a defect — the same "synthetic now, real sensor later"
discipline as `tracking`. Pipeline: **scan → coarse init (centroid) → ICP → pose + deviation**.
`estimate_pose` returns metres+quat for `tracking.observe`; `deviation` returns the as-built
signed-distance map + a tolerance verdict.

**Two findings worth keeping:**
- **Coverage, not residual, makes a scan trustworthy.** A single flat view of a plate is
  *geometrically degenerate* — ICP settles a plate-thickness off in Z **with a low RMSE**
  (5 mm pose error, deceptively good fit). Multi-view (turntable / arm sweep) recovers the
  pose exactly. A low residual is necessary, not sufficient — check observability.
- **ICP is local** — it needs a coarse initial pose (nominal / centroid). Arbitrary-pose
  parts need a global-registration front end (feature matching / FPFH) before ICP; that's the
  next capability if parts arrive in unknown orientations.

## Method space (with the project's triage lens)

Reward **deterministic, digital-pattern, CAD-registerable** methods; be wary of stochastic
texture-dependent reconstruction *as a metrology source* (fine for capture, not verification).

| Method | Accuracy | Cost | Lens verdict |
|---|---|---|---|
| **Laser-line triangulation** (line laser + camera, swept) | ~20–50 µm | $ | ✅ deterministic; ideal mounted on the arm |
| **Structured light** (projected pattern + camera, turntable) | ~50–100 µm | $$ | ✅ digital-pattern; best bench metrology |
| **ToF / stereo depth cam** (RealSense-class) | ~1–5 mm | $ | ~ coarse pose + scene collision, not metrology |
| **Photogrammetry** (multi-photo SfM) | scale-ambiguous | $ | ~ stochastic, texture-dependent — capture, not verify |
| **Touch probe / CMM** (arm-with-a-probe) | ~10 µm, sparse | $–$$$ | ✅ gold accuracy; the local-datum idea, few points |
| **LiDAR** | cm | $$ | ✗ range/scene scale, too coarse for parts |
| **Confocal / interferometry** | sub-µm, tiny FOV | $$$$ | niche (surface finish) |

## The arm IS the scanner (the on-thesis path)

Mount a **line laser or a small depth cam on the SO-101 wrist** (or as a tool-changer tool).
The arm's FK (`bridge/` Placo) gives the sensor pose per frame; sweep a **scan toolpath**
(`orchestration/toolpath.py` — a scan is just another operation path) over the part and fuse
points via the known poses into a cloud in the workcell datum. This unifies toolpath +
tracking + scanning, and turns the arm into a poor-man's CMM. The one new calibration item is
the **hand-eye transform** (sensor vs. flange) — identity until measured, exactly like the
sim-world↔arm-base transform (see `calibration/`).

## Next steps

- Global registration front end (FPFH/RANSAC) so unknown-orientation parts register without a
  good init.
- A real sensor adapter (RealSense / a line-laser rig) feeding `register_to_cad` — flips the
  synthetic scan to a measured one, same seam as the tracking observation source.
- Deviation → calibration **writeback** as a reviewable geometry diff (mirror `measure.py`).
- Fuse an arm-swept multi-pose scan through the FK/hand-eye chain (the CMM path above).
