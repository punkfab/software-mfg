# North star — Manufacturing for Design

The strategic frame this repo is building toward, and the vocabulary for it. These are
aspirational north-star ideas: some are load-bearing today, some are direction. Each term is
kept honest by pointing at the capability that already demonstrates it — buzzwords with a gate
behind them.

## The shift: point design → space design

Traditional **Design for Manufacturing (DFM)** asks a one-shot question at the end: *can you
make this part?* — the design bends to fit the machine. When simulate-and-verify costs almost
nothing, that inverts. You stop designing *a* part and start designing *a space*: sweep a
family of variants, build and check each in software, and let the constraints pick the
survivor. Manufacturing becomes the **inner loop of a search**, not a terminal gate. The factory
is a subroutine of the design, not a constraint you contort around.

The proof that hardware iteration can approach software cadence is already out there — rocket
shops that 3D-print sub-scale engines and hot-fire them in instrumented test cells on a tight
loop. Cheap build + instrumented verify + fast turn = iterate atoms like code.

## The lexicon (with the capability that earns each term)

| Tier | Term | Meaning | Earned by (in this repo) |
|---|---|---|---|
| **Spine** | **Manufacturing for Design (MfD)** | The inversion of DFM — the machine serves the design search. | The whole stack: define → sim → verify is one loop (`make check`). |
| **Philosophy** | **Design as Search** | You don't draw a part; you search a space, and the winner is *verified*, not asserted. | The omni-wheel pitch×gap sweep converging on a feasible design (`parts/_omni.py`, `omni_check.py`). |
| **Method** | **Sweep the envelope** | Explore hundreds of variants for near-zero cost; map the whole feasible region, not one point. | The `MOUNT_R × gap` sweep that found where rollers stop self-colliding. |
| **Discipline** | **Test-Driven Fabrication (TDF)** | Write the constraint (the gate) first; the fab converges until the build goes green. | Every `scripts/*_check.py` gate is a test the design must pass before it's "done". |
| **Trust** | **The simulation is a cache of reality** | Every parameter is measured on real hardware, stamped for freshness, re-checked before it's trusted. | `calibration/` staleness (FRESH/STALE/EXTRAPOLATING/PREDICTION) + measurement writeback. |
| **Guarantee** | **Verified by construction** | Some constraints aren't checked after the fact — they're made impossible to violate by how the part is built. | The carve-out omni hub: roller clearance is guaranteed by subtracting the roller envelopes. |

Supporting phrases: **Manufacturing CI** / **CI-CD for atoms** (the gate suite); **hardware at
software cadence** (the loop); **continuous calibration** (the reality leg).

## Why it isn't vapor — three things the repo already did

- **Swept to a feasible design, verified by a gate.** The omni wheel wasn't traced from a photo;
  its pitch/gap were *swept*, the interference checker *rejected* the self-colliding ones, and
  the survivor is proven to spin free. That's Design as Search + TDF, in miniature.
- **Verified before cutting metal.** The mobile-base model caught that a real off-the-shelf
  chassis fails three ways *before* it was bought (`sim/mobile_base.py`, `MOBILE_BASE.md`) —
  the value of sweeping in software is the mistakes you never pay for.
- **A green build means a good part, not a hopeful one.** Calibration staleness makes the sweep
  trustworthy: results carry FRESH/PREDICTION stamps so an unvalidated variant can't masquerade
  as a proven one.

## Where this points

The product direction is **generative sweeps as a first-class operation**: declare a parameter
space + the constraints (gates), let the system fan out variants, verify each, and return the
feasible envelope + the recommended point — the natural extension of Phase 5 (generative DFM)
and the op-graph. `punkfab` is the commercial face of this thesis; this file is its charter.
