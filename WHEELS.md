# Omni Wheels — source analysis + what we built

Two things kept getting conflated, so this file separates them:

1. **The real wheels** — the downloaded XRP Kiwi-bot files, and what measuring them told us.
2. **What we built** — the parametric / IR models in this repo (which are *approximations* of, or
   *unrelated to*, the real wheel).

---

## 1. The real wheel (source files)

Everything downloaded is the **XRP "Omni-Directional Kiwi Bot" drive base** kit (`~/Downloads/
xrp-omni-directional-kiwi-bot-drive-base-model_files/`). It ships in two forms:

| file | what it is | key facts |
|------|-----------|-----------|
| `kiwi-omni-directional-bot-v10.step` | **the merged whole-bot STEP** — the entire drive base as one file | 215 × 195 × 70 mm, **84 solids**; the thing you'd have to split to reverse-engineer to parts |
| `Omni Wheel/omni-wheel-shell.step` (= `kiwi-… - Omni Wheel Shell.step`) | **the one actual standalone wheel STEP** (B-rep) | OD 68, width ~39, ~43 k mm³ |
| `Omni Wheel/omni-wheel-shell.stl` | the same wheel as a **mesh** | 55 k triangles — no B-rep, no features |
| `omni-wheel-chassis.stl`, `…-drive-base-….pdf` | chassis + build instructions | — |
| `~/Downloads/omniwheel-edited.step` | **your hand-edit of the wheel** | OD 68, width 37.6, **36 k mm³** (hollowed further) — the current target |

So there is **one wheel** (XRP, OD 68, 8 rollers) that exists as: a mesh, a standalone B-rep, a
part inside the merged bot, and your edited B-rep.

## 2. Analysis — what measuring the XRP wheel told us

Measured off `Omni Wheel Shell.step` (OCCT face classification; download's axis = Y):

- **OD 68**, width ~39, ~43 k mm³ solid (the shell; your edit hollows it to ~36 k).
- **Rollers: 8 total = 2 rows × 4, staggered 45°.** Found via 16 × Ø3.2 pin bores → 8 coaxial
  fork-pairs; roller-axis pitch **MOUNT_R ≈ 30**.
- **Continuous contact** — *empirically*: 100 % OD coverage, 0° gap. Per-roller arc ≈ **±28°**,
  because the barrels **overhang their pins ~2.6×** (pin span implied only ±11°). This is why our
  old analytic continuity check (`atan(HALF_L/MOUNT_R) ≥ 90/N`) was wrong — see the geometric
  continuity check in `scripts/omni_check.py`.
- **Drive:** Feetech **STS3215** servo. Horn from datasheet **ST-3215-C047** §11: horn OD Ø19.95,
  hub Ø9, bolt circle **Ø14**, **4 × Ø3.2 (M3)**, spline **25T / Ø5.9**, retaining screw M3×6.
- **Format reality:** the STL is a mesh (rejected by any B-rep tool); the STEP is B-rep but the
  wheel is a **revolve + a circular pattern of freeform (bspline/sphere/torus) roller pockets** —
  *not* the 2.5D-prismatic class, so automatic STEP→IR recognition returns UNSUPPORTED (correct).

## 3. What we built (models in this repo)

Three *distinct* wheel models live here. **Only #2 is the XRP wheel** — #1 is an unrelated design
and #3 isn't an omni wheel. (This was the main source of confusion.)

| # | model | files | rollers / OD | relation to the XRP wheel |
|---|-------|-------|--------------|---------------------------|
| 1 | **parametric omni** | `parts/_omni.py` + `omni_hub.py` + `omni_roller.py` | **2×5 = 10, OD 60** | **UNRELATED** — grey-box reverse-engineered earlier from a *photo* of a different wheel. Gated by `make omni-check`. |
| 2 | **`kiwi_wheel` (IR)** | `scripts/ir_local.py:kiwi_wheel` → `exports/freecad/kiwi_wheel.FCStd` | **2×4 = 8, OD 68** | **the XRP wheel**, authored as featuretree IR + an STS3215 servo mount. |
| 3 | **`base_wheel`** | `parts/base_wheel.py` | solid tire, no rollers | a **from-scratch** diff-drive drive wheel — not omni. |

**#2 (`kiwi_wheel`) is a hand-authored IR reproduction**, not a recognizer output. It is the wheel's
**body-of-revolution envelope + roller cavities**, i.e. a *solid* approximation (~83 k mm³ with the
servo mount) — the IR cannot express the real wheel's hollow spoked interior. What matches the real
wheel exactly: **OD 68, 8 staggered roller pockets, MOUNT_R 30**, and the STS3215 mount. It adds a
servo-drive interface not in the download: a Ø22 standoff boss (roller-to-servo clearance), a Ø20.5
horn recess + Ø14/4×M3 bolt circle, and a back-access counterbore (short screws).

## 4. Tooling this drove (in ../featuretree, by reference)

Reverse-engineering the XRP wheel is what forced these featuretree features into existence:

- **`revolve`** + **XZ-plane profile sketches** — bodies of revolution (the wheel disc/hub).
- **`polar_pocket`** — a ring of tangent cylindrical pockets (the roller cavities).
- **`step_recognize`** — STEP→IR for the 2.5D-prismatic class, with fail-loud verification (it
  correctly refuses this wheel).
- **geometric continuity** (`scripts/omni_check.py`) — measure OD coverage on the assembled wheel
  instead of the fragile analytic proxy.

## 5. Open decisions

- **#1 vs #2**: the OD 60 / 10-roller parametric wheel (#1) is a *different physical wheel* from the
  XRP OD 68 / 8-roller wheel (#2). Decide whether both are wanted, or retire one.
- **`kiwi_wheel` fidelity**: it's a solid approximation; the hollow spoked interior is out of the
  IR's reach. Reconcile its boss/width/mount dims to `omniwheel-edited.step` if an exact match to
  your edit matters.
