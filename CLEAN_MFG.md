# Clean manufacturing — the honest green thesis

The sustainability case for Manufacturing for Design (MfD), captured as a working thesis and an
investor-facing brief. It bridges **punkfab** (software-defined fabrication) with a
**clean-energy investing** thesis. The discipline is the same as the rest of the repo: every
claim has a number or a caveat behind it. **Flag the tensions first — that honesty is the
credibility.**

One line: **clean manufacturing = less matter, less freight, fewer loops-to-virgin — all
*verified*.**

## The anchor stat: making + moving ≈ two-thirds of global energy

Global final energy consumption splits roughly: **industry ~38%**, **transport ~28%**,
buildings the rest. MfD has a lever on the first two — the energy of *making* things and the
energy of *moving* them.

- Honesty note: "industry" (~38%) includes mining, agriculture, and construction. **Manufacturing
  is the bulk of it, not all of it.** Defensible phrasing: *"making things uses close to 40% of
  the world's energy, and manufacturing is most of that."* (EIA: industry ≈ 37–38% of global
  delivered energy, 2019, rising toward ~40% by 2024.)

## The frame: material efficiency + iteration efficiency (not magic-clean)

MfD is not "clean" by fiat — a robot cell still uses electricity. It is a **resource-efficiency**
story: it decouples design *iteration* from material and logistics *throughput*.

> Traditional prototyping is iterate-by-scrap. Software-defined fab is iterate-by-simulation.
> **The greenest part is the one you never built wrong.**

## Lever 1 — Making (industry ~38%)

- **Scrap avoided.** Sweep 100 variants in sim, build 1 → save the embodied energy + material of
  the 99. (Proven: the omni-wheel sweep; the mobile-base model that caught a bad chassis *before
  purchase*.)
- **Right-sizing / lightweighting.** "Map the envelope" finds the minimum-material design that
  still passes — no over-building "to be safe."
- **No hard tooling.** Molds/dies are huge embodied energy and get *scrapped* on redesign.
  Software-defined cells reconfigure in code; a redesign scraps nothing.
- **Additive / near-net-shape + recycled feedstock.** Foil cells: recycled aluminum ≈ **5%** of
  virgin energy; layer-forming uses only the material in the part (vs machining-from-billet,
  mostly chips).

## Lever 2 — Moving (transport ~28%)

- **Distributed / on-demand production.** Parts-as-code made where/when needed collapses the
  long-haul freight leg — ship a file, not a pallet.
- **Digital inventory replaces warehoused-and-shipped stock** — kills both the shipping and the
  overproduction waste (make-then-discard).
- **Lightweighting compounds in transport.** In anything that moves, every gram saved is fuel
  saved for the whole service life. Right-sizing pays twice: once in material, once in every mile.
- Honesty: distributed ≠ automatically lower-transport (a mega-factory has scale efficiencies).
  Claim: *we cut the freight, the warehouse, and the scrap* — net-positive when those dominate.

## Lever 3 — Looping (design for remanufacture / verified circularity)

The moat. Circular economy stalls because **you can't trust second-life material** — the fatigue
state of a used part, the contamination in regrind, the remaining life. So people default to
virgin. The bottleneck is *verification*, and verification is this repo's differentiator
(calibration/staleness + scanning + provenance).

> punkfab's circular pitch isn't "we recycle." It's **"we recycle and *prove* the second-life
> material still meets spec."** Verified circularity.

**Close the tightest loop you can *prove*** (the R-ladder — tighter loop retains more embodied
energy):

| Loop (tightest → loosest) | Retains | MfD capability |
|---|---|---|
| **Reuse** as-is | ~all embodied energy | scan → deviation vs CAD = "still in spec?" gate |
| **Repair / Remanufacture** | the shape's energy (~80%+) | scan the wear map → corrective toolpath → re-verify |
| **Recycle** material | material only; shape's energy lost | foil cells (recycled Al ≈ 5% of virgin), near-net-shape |
| **Recover** (burn) | just heat | last resort — name it as failure |

Lesson: **don't recycle what you could remanufacture** — recycling melts away the energy locked in
the *geometry*. Scanning lets you decide whether the tighter loop is still safe — the call nobody
can make today without a trust layer.

- **Outputs as inputs** ("the shop eats its own scrap"): failed prints reground/refed, foil offcuts
  re-laminated, swarf remelted; the foil "machines that make machines" loop. The honest metric is a
  **mass balance** — with recovery yield Y and scrap rate s, what fraction of virgin feedstock do
  you displace per cycle? (Planned: `sim/material_loop.py` — the gate behind this claim.)
- **Material passport.** The provenance ledger (PLAN Phase 10) — every part carries a signed record
  of composition, cycle count, measured state. *Every part remembers where it's been.* Regulatory
  tailwind: the EU **Digital Product Passport** is mandating exactly this (batteries, textiles,
  electronics) — so the ledger is a compliance product with a deadline, not just hygiene.
- **Second-life as PREDICTION.** Recovered material/parts enter untrusted (`PREDICTION`) and earn
  `FRESH` only after characterization — the calibration vocabulary carries the honesty in one word.

## The honesty (say these first)

- **It's a spiral, not a circle.** Most recycling *downcycles* — quality drops per loop (entropy of
  mixing, contamination). No infinite loops. Claim: characterized second-life material good for the
  number of cycles it can hold spec.
- **Recovery isn't free** — collection, sorting, reprocessing cost energy; the win is net and
  biggest at the tight end of the ladder. Quantify, don't assert.
- **"Clean" is grid-dependent.** Claim: *electrified and sitable near clean power*, not zero-carbon.
- Robot cells + simulation use energy. Win is net (avoided scrap ≫ sim compute) — measure it.

## The green lexicon (extends NORTH_STAR.md)

- **Design for Remanufacture (DfRe)** — design a part to be recovered, *re-verified*, and re-run,
  not just shredded.
- **Verified circularity / provable loops** — the moat.
- **Material passport** — the ledger as circularity infrastructure ("every part remembers where
  it's been").
- **Second-life as PREDICTION** — recovered inputs untrusted until characterized.
- **The tightest verifiable loop** — the operating principle.
- **Dematerialized iteration** — decoupling design iteration from matter + logistics.
- **Outputs as inputs / "the shop eats its own scrap."**

## The fund correlation

Two directions, both underwritable because every claim is caveated:

1. **Decarbonize a hard-to-abate sector.** Industry is ~38% of global energy; electrified cells +
   minimal scrap + sitable-near-clean-power is a real lever on the making of things.
2. **Enable the portfolio.** Clean-energy + critical-materials hardware (electrolyzers, batteries,
   rare-earth/battery recovery, remanufacturing) is slow/expensive to iterate and *all needs the
   same thing: trust in recovered material.* MfD's characterize-and-passport layer is the enabling
   tech across that whole theme — plus the DPP regulatory wave. punkfab isn't just *a* green
   company; it's infrastructure that speeds up every hardware bet the fund makes.

## Status / next builds

Evidence today: scrap-avoided (omni sweep, mobile-base pre-purchase catch), calibration staleness
(the trust stamp), foil cells (recycled feedstock), scanning (wear assessment / repair-not-replace).
Roadmap: `sim/material_loop.py` (mass-balance / virgin-displacement, the gate behind "outputs as
inputs"), the provenance ledger / material passport (Phase 10), a punkfab-site "clean manufacturing"
section.

## Sources

- EIA — Industrial sector energy consumption (industry ≈ 37–38% of global delivered energy):
  https://www.eia.gov/outlooks/ieo/pdf/industrial.pdf
- IEA — Industry (Energy Efficiency 2025): https://www.iea.org/reports/energy-efficiency-2025/industry
- IEA — Global Energy Review 2025: https://www.iea.org/reports/global-energy-review-2025/electricity
