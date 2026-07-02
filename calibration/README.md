# calibration — the round-trip's reality leg

Hardware CI has a gap software CI doesn't: you **test a simulation** but **ship a
physical part**. A sim is only trustworthy where its parameters have been anchored
to reality. This package is that anchor — and the honesty signal that says when a
green sim has drifted out of trust.

Three loops run in this project; this one keeps the other two honest:

1. **design round-trip** — declarative source ↔ editable representation (`featuretree/`)
2. **sim-CI** — design → simulate → verdict (the `*-check` gates); fast, cheap, but validates a *model*
3. **reality calibration** — physical build → measurement → parameter update (**here**)

## The idea

> The sim is a **cache of reality**. Caches need invalidation.

A calibrated parameter is not a number — it's a number plus *how far you can trust
it*: value/σ (distributional, because parts scatter), the operating point it was
measured at, the envelope over which it's believed, and which physical build last
confirmed it. `store.json` is that vector, git-tracked; the sim reads it (see
`sim/press_cell.py`), calibration writes it.

Every sim result carries a **staleness stamp**:

| verdict | meaning | gate action |
|---|---|---|
| `FRESH` | recently anchored, in-envelope | trust green |
| `STALE` | too many design iterations since reality confirmed it | re-anchor before trusting |
| `EXTRAPOLATING` | ran outside where the model was ever validated | green is a guess |

Staleness is counted in **builds-since-anchor**, not wall-clock — the target is
moving, and what matters is how many iterations you've run since reality last spoke.

## The writeback is a pull request

A physical build doesn't silently overwrite the model. `ingest()` compares what the
part *did* to what the sim *predicted* and returns a `ParamDiff` (old → new, the
residual the sim got wrong, updated σ) for review. `apply()` commits it and advances
the build clock. Same discipline as the featuretree IR round-trip: the reverse leg
produces a reviewable diff, not a mutation.

## Try it

```
make calib        # walk the whole loop: staleness stamp (3 verdicts) -> ingest -> diff -> re-anchor
make calib-check  # gate: the press sim runs on FRESH, in-envelope params, with margin
```

## Files

- `store.py` — `Parameter`, `CalibrationStore`, `staleness()`
- `measure.py` — `Measurement`, `ingest()` → `ParamDiff`, `apply()`
- `store.json` — the tracked parameter vector (seeded: press force/seat depth, bend springback)
