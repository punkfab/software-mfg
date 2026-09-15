# Competitive + prior-art scan — Arche Industries ("Smith") vs. software-mfg

_Scan date: 2026-09-15. Method: three adversarial researchers (company intel · AI-CAD prior art ·
DFM/marketplace/FTO), each briefed to prove our angle is already occupied. Patent independent claims
read verbatim where cited. Every citation tagged verified / secondhand / unverified below._

## Executive verdict

**The shared "requirements → physical object" thesis is not novel and not defensible as IP — it is a
crowded, actively-converging space, and Arche is a direct thesis-twin already shipping the bundle.**
Stop pitching "collapse the distance from requirements to object," "NL→CAD-as-code," or "in-loop sim"
as new. Our defensible ground is **positioning + execution**, on three specific wedges, only one of
which is even framing-novel:

1. **Owning the physical robotic cells** (open, cheap, composable) — a defensible *wedge/niche*, not
   patentable; and it's the cleanest separator specifically vs. Arche, who does **not** own fabrication.
2. **The calibration trust-contract** (the FRESH/STALE/EXTRAPOLATING staleness stamp as a first-class,
   user-facing interface) — the only genuinely **open framing** found; the underlying math is not.
3. **Honest clean-mfg / material-efficiency angle** (see `CLEAN_MFG.md`) — adjacent, uncontested by
   Arche's manifesto, but table-stakes on the provenance side.

## Who Arche is (as of the scan)

- Real, live, **very young** — public footprint ≈ **June 2026** (X @archedotco created 2026-06-08;
  site © 2026; domain re-registered 2026-06-04). Waitlist-gated app. [verified — whois + X snowflake]
- Positions as **"Cursor for physical design"**: NL + napkin-style geometry → CAD with **Python as the
  source of truth**, live GPU structural + CFD, **DFM Mode** (print-orientation/overhang/wall solver;
  sheet-metal flat pattern + bend tables w/ per-material K-factors), **Scrapyard** (indexes supplier
  catalogs, agent-queryable, closes to a shipped part). [verified — homepage + /manifesto + /login]
- **Manifesto names Protolabs, Xometry, Fictiv, McMaster as targets**; bets "the frontier is physical."
  Notably, Arche's "Build Real" is still **catalog/supplier-mediated — they do not own fabrication.**
- **Founders, funding, team, customers: all undisclosed / unverifiable** from free sources. No named
  founder anywhere; no indexed raise (YC/TechCrunch/Tracxn genuine-zero). **Crunchbase 403 — a raise
  cannot be positively ruled out.** Zero patents under assignee "Arche Industries." [verified/blocked]
  - ⚠️ Do **not** attribute the search-engine "founders Mahalingam Ramasamy / Bobby Md" result — that is
    a *different* Sunnyvale consultancy (archecompany.co), confirmed mismatch. [verified]

## What is definitively CLOSED (do not claim as novel)

| Claim | Closest prior art | Closeness | Source |
|---|---|---|---|
| NL → CAD-as-code | **Zoo/KittyCAD** (emits KCL); **Adam / AdamCAD** YC W25, **$4.1M seed Oct 2025**, emits OpenSCAD; NeurIPS'24 **Text2CAD**; **Text-to-CadQuery** (Python) | IDENTICAL | zoo.dev · techcrunch.com (Adam raise) · neurips.cc/…/96571 · arxiv 2505.06507 [verified] |
| Real-time in-loop GPU FEA/CFD | **Ansys Discovery Live** (since ~2017); Luminary Cloud; PhysicsX; nTop 3.0; SimScale | IDENTICAL | ansys.com/…/ansys-discovery [verified] |
| The *combination* (NL-code-CAD + in-loop sim) | **Neural Concept** AI Design Copilot (CES Jan 2026); arXiv **"Physics-in-the-Loop… Validated CAD"** (2026) | VERY CLOSE | neuralconcept.com · arxiv 2605.19717 [verified] |
| DFM auto-opt (orientation, K-factor unfold, process checks) | **Xometry** Instant Quoting + ML DFM; **Fusion 360** native sheet-metal + DFM; aPriori | IDENTICAL | xometry.com/machine-learning-for-manufacturing · autodesk.com/…/design-for-manufacturing [verified] |
| Live supplier-catalog parts as CAD primitives | **Fusion "Insert McMaster-Carr"** + Autodesk–TraceParts; **CADENAS PARTsolutions** | IDENTICAL | help.autodesk.com · partsolutions.com [verified] |
| Forkable/remixable design marketplace + live 3D + order | MakerWorld, Thingiverse remix, GrabCAD, Thangs, Wikifactory | VERY CLOSE | makerworld.com · thingiverse.com [verified] |
| One-click "publish immutable version → order the part" | Fusion **"Make"** → Protolabs; Xometry/Protolabs instant order | VERY CLOSE/IDENTICAL | ketiv.com/…/fusion-360-and-proto-labs [verified] |

**The combination is thin white space measured in months, not a moat.** Two independent efforts
(Neural Concept product + the Physics-in-the-Loop paper) already sit on the seam.

## What SURVIVES — our three differentiators, graded

**Claim 5 — Calibration "cache-of-reality" (staleness stamp + measurement writeback + grey-box ID).**
The *mechanism* is OCCUPIED: grey-box system ID for mfg digital twins, self-improving reality-to-sim
gap closure, online Bayesian/particle-filter twin calibration with parameter writeback are all
published; **SmartUQ** commercializes UQ+twin calibration. [verified — tandfonline 10.1080/0951192X.2024.2386980;
sciencedirect S2405896322001823; arxiv 2011.09810; smartuq.com]
_Open sliver:_ the explicit **FRESH/STALE/EXTRAPOLATING staleness stamp as a first-class user-facing
trust contract gating downstream use** — **not found shipping anywhere.** This is a **framing edge, not
a technical moat**, but it is our cleanest white space and the sharpest way to differentiate the sim story.

**Claim 6 — Signed provenance receipts / material passport (ed25519, EU DPP). MOST CONTESTED.**
Direct blocker-grade art: **Moog "VeriPart" — US11107168B2** (filed 2016/17, active ≈2037): customer
requirement → geometry → print, **each step signed with a private key, recorded to a distributed
transaction register, unique cryptographic product code physically marked on the part.** That is our
concept. Plus a mature verifiable-credential DPP industry (cheqd, Spherity, TraceX) under eIDAS-2 QES.
[verified — Google Patents US11107168B2 independent claim; cheqd.io; spherity]
_Daylight (narrow, design-around not differentiator):_ ed25519 **local trust-ledger NOT on a
blockchain/DLT** (Moog's independent claim recites a "distributed transaction register"), and receipts
tied to **our own cell measurements**. **Treat provenance as table-stakes, not a moat.**

**Claim 7 — Owning composable robotic cells. Category OCCUPIED, niche DEFENSIBLE.**
**Bright Machines** "software-defined microfactory" owns the space and the patents — **US11520571B2**
(+ family US12056476, US11858134, US12151373): modular reusable cells + recipe-based declarative
programming + autocalibration + digital twin, active ≈2040. Also Machina Labs, Vention, Formic, Path,
Divergent. [verified — Google Patents independent claims; machinalabs.ai; vention]
_But none serves our bet:_ cheap, **open-hardware, composable** cells (SO-101 arm + printed tool-changer
+ LeKiwi omni base) doing **low-volume / one-off** builds — incumbents are capital-heavy high-volume.
Defensible as a **wedge and price-point**, not as IP. **This is also the cleanest separator vs. Arche
specifically, who owns no fabrication at all.**

## FTO watch-items (independent claims read)

- **US11107168B2 — Moog (VeriPart)** — signed-AM-provenance-to-ledger + physical crypto code. Active
  ≈2037. → gates Claim 6. Design-around = non-DLT local ed25519 ledger.
- **US11520571B2 (+ family) — Bright Machines** — software-defined modular cells. Active ≈2040. →
  watch for Claim 7 if we ever productize "cell as code."
- **US9747394B2 / US10061870B2 — Xerox/PARC** — 3D-printability slice-sim + automated
  metrology/model-correction. Active ≈2036. Narrow method claims, **not broad blockers**, but adjacent
  to Claims 1/5 — don't recite their specific methods.

## Verification debt / caveats

- **Crunchbase 403** — Arche funding not positively confirmed *or* denied. A raise may exist unindexed.
- Founders unknown → **no inventor-name patent search possible** for Arche; any filing would likely be
  pre-publication (18-mo lag) anyway.
- Items tagged [secondhand]/[unverified] above (aPriori, Leo AI, Navier AI, some Autodesk-adjacent
  patents) were named in search summaries but not independently fetched.
- This is an amateur sweep — good enough for **positioning and "what to claim"**; a **filing** decision
  on Claim 5's staleness-stamp framing would need a professional search.

## Recommended next actions (ranked)

1. **Reposition the narrative.** Lead with the **calibration trust-contract** (staleness stamp) and the
   **open composable-cell price point / low-volume niche** — not "requirements→object." Update
   `NORTH_STAR.md` framing accordingly.
2. **Demote provenance to table-stakes** in the pitch; keep the non-DLT ed25519 design as a feature,
   not a headline. Note the Moog patent so we never recite its method.
3. **Sharpen the anti-Arche wedge:** we *make the part on owned cells*; Arche is a software front-end to
   supplier catalogs (their own manifesto targets Protolabs/Xometry/McMaster — the same outsourced
   supply chain Scrapyard rides on). Own the fabrication story.
4. **Watch Arche** (X @archedotco, waitlist) for a funding announcement or a named team — the one open
   question free sources can't close.

---

## Watchlist — adjacent AI-CAD players (tracked, not full-scanned)

The "vertical AI-native CAD" category is filling in across **both geometry kernels and domains** at
once — the generation front-end is commoditizing on every axis. This *reinforces* the verdict above:
cede CAD generation, win downstream (owned cells + calibration + verified assembly).

| Player | What / kernel / domain | Distance from our north star | Notes (as of scan) |
|---|---|---|---|
| **Arche — "Smith"** | code-CAD (Python), mechanical; +sim +DFM +Scrapyard | **CLOSEST — thesis-twin** | full scan above; owns no fabrication |
| **Formas — "Cartesian"** (formas.ai) | **B-rep/NURBS**, arch + product; "Anything to 3D"; Rhino .3DM / SketchUp .SKP, IFC planned | **FURTHEST** — pure geometry gen, **no sim/fab/assembly/calibration** | closest thing to a **featuretree** competitor (editable B-rep, "no mesh approximation"). Preview 2026-09-18, waitlist. Team/funding undisclosed. [verified — formas.ai/cartesian] |
| **Embedr** (embedr.app) | code, **embedded/PCB + firmware**; AI-native Arduino IDE, KiCad-native, datasheet→knowledge agent | adjacent — competes only with the **PCB/firmware corner** (pcb-layout / circuit-sim), not the mechanical core | potential build-vs-buy for the firmware leg, not a head-on rival. [verified — embedr.app] |

**Category takeaway:** three entrants, three different geometry representations (code / B-rep / code)
and three domains (mechanical / arch+product / embedded). Expect more (RF, optics, fluidics). Our
domain-knowledge layer is *manufacturing-process + calibrated reality*, which is harder to replicate
than a geometry kernel or a datasheet corpus — that is where the defensibility lives.
