# Process design space — a survey of fabrication archetypes

software-mfg is a bet that you can *software-define* a fabrication process: describe
it declaratively, simulate it, verify it, and improve it in a fast loop. Not every
physical process takes that bet equally well. This doc maps the processes we've
considered — the ones built, the ones worth building, and the ones to consume as a
black box or reject — and scores them by how well they fit.

It's a living map. Add rows as new processes come up.

## The triage lens

A process is a good fit for software-mfg to the degree it has:

1. **Deterministic forward model** — given the program, you can *predict* the output
   geometry, not just measure it after. Wire bending: yes (a polyline). EDM: no (a
   stochastic spark).
2. **Small or calibratable variance** — the sim-to-real gap collapses to a few
   parameters the [calibration layer](./calibration/README.md) can anchor, not a
   dozen coupled empirical knobs.
3. **Ubiquitous / bootstrappable feedstock** — cheap, everywhere, ideally usable to
   make more tooling (foil, wire).
4. **Replicable machine** — ideally repurposes commodity hardware, so the machine
   that makes machines is itself makeable.

Scores are H / M / L, deliberately qualitative.

## The map

| Process | What it does | fwd-model | variance | feedstock | machine | Verdict |
|---|---|:--:|:--:|:--:|:--:|---|
| **Wire bending** | 1D stock → space curve | H | H | H | H | **built** — `sim/wirebender_cell.py` |
| **Foil former** | sheet → folded stiff profile | H | H | H | H | **built** — `sim/foil_former.py` |
| **Foil LOM** | stack + bond + cut foil → solid | H | M | H | M | **built** — `sim/foil_lom.py` |
| **Press / insert** | self-reacting axial force | H | H | H | H | **built** — `sim/press_cell.py` |
| **Hot-melt gluing** | dispense adhesive bead along a path | H | M | H | H | **built** — `parts/glue_*.py`, `sim/glue_cell.py` |
| **Pulse / spark forming** (EHF / EMF) | high-rate die-conform of sheet | M | M | M | L | **candidate** — deletes springback |
| **VFA impact welding** | solid-state weld foil layer-to-layer | L | M | H | L | **candidate** — full-density LOM |
| **Cold spray** | supersonic solid-state powder build-up | M | M | M | L | **candidate** — dense bulk Al (not plasma) |
| **Electrophotographic powder print** | electrostatic-patterned powder + fuse | H | M | M | H | **candidate** — bitmap program, commodity HW |
| **Plasma spray** (thermal) | molten splat coating | L | L | M | L | coatings only (porous, oxidized) |
| **PVD sputtering** (HV plasma) | condensed thin film | M | M | M | L | coatings / metallization only (µm, slow) |
| **EDM** | subtractive spark erosion | L | L | M | L | **rejected** — removes, doesn't form; perforates foil |

---

## Forming

**Pulse / spark forming (electrohydraulic EHF, electromagnetic EMF).** A capacitor
bank discharges — a spark across electrodes in water (EHF) or a pulsed coil's Lorentz
force (EMF) — and the pressure/impulse slams a blank into a die at high strain rate.
For foil this is arguably *better* than the mechanical former because the die sets the
shape: **springback ≈ 0** (inertial die-ironing), no mechanical tool to wrinkle the
membrane, single-sided tooling. Aluminum's high conductivity makes it the ideal EMF
material — but very thin foil *under-couples* (high resistance → it tends to vaporize
rather than move), which is exactly the failure mode the next entry exploits.

**Vaporizing Foil Actuator (VFA) — foil as the driver, not the workpiece.** Dump a
capacitor bank through a thin aluminum foil; it electrically explodes in microseconds
and the pressure burst drives a separate workpiece into a die, or does **impact
welding** (solid-state collision welds). The self-referential payoff for us: **VFA
impact welding is a bonding step for the foil LOM** — weld stacked layers to full
density instead of gluing them, retiring the `foil_bond_shear_mpa` delamination gate
and the stack's anisotropy. Foil's own energy bonds foil into a solid.

Both are *hard multiphysics* (discharge → EM/shockwave → transient plasticity → die
contact — LS-DYNA territory, not MuJoCo's rigid-body wheelhouse), but they abstract
cleanly as a one-shot op: `blank + die + energy → die-conforming part, springback ≈ 0`,
with a formability / tear limit as the key calibrated parameter. Caveat: kJ/kV banks
with explosive discharges — lab equipment, not benchtop-benign.

## Deposition

**Deposited aluminum is not foil.** You can deposit aluminum with HV plasma, but the
result is a different material — porous splat (thermal spray) or thin columnar film
(PVD) — lacking foil's wrought ductility and full density. Deposition and foil-stacking
are *rivals* for building parts, not partners.

- **PVD sputtering / evaporation** (true HV glow-discharge plasma): dense-ish but
  ~µm-in-minutes and in vacuum → **coatings / metallization only** (aluminized film,
  mirrors), hopeless for bulk.
- **Plasma spray** (thermal torch): fast and thick but porous (few %–15%), oxide-laden,
  splat-anisotropic → **coatings / rough build-up**, not wrought metal.
- **Cold spray** (*not* plasma — supersonic gas, solid-state impact bonding): dense
  (>99%), near-wrought, low-oxide → the real answer for **dense bulk aluminum from
  cheap wire/powder**, and the additive cousin of VFA impact welding. The catch: the
  nozzle/gas hardware is harder to replicate than a roll of foil — a bootstrapping
  tension (better freeform geometry, worse feedstock-ubiquity and machine-replicability).

The one productive hybrid: use plasma/arc (or diffusion / ultrasonic) energy not to
*deposit* but to **fuse** stacked foil layers — deposition energy in service of the
foil stack, not replacing it.

## Patterned powder — electrophotographic ("xerographic") 3D printing

The newest thread, and a strong fit. Charge a **photoconductor** film uniformly, expose
it to a **light image** so the lit areas discharge → a **latent electrostatic image**,
develop it with charged powder (toner), transfer the patterned layer to the build, and
**fuse**. Repeat. This is a laser printer turned into a 3D printer.

- **Prior art:** Evolve Additive Solutions' **STEP** (Selective Thermoplastic
  Electrophotographic Process) does exactly this at production scale — a full layer
  every 2–4 s (~50× SLS), 100% dense thermoplastic parts.
- **The reuse twist (ours):** drive it with an **MSLA LCD as the layer-at-once UV
  exposure engine** (instead of a scanning laser) and a repurposed laser-printer
  photoconductor + corona. The layer program is a **bitmap stack — identical to MSLA
  slicing** (H on software-definability), and it repurposes **two commodity consumer
  machines** (H on machine-replicability).
- **The UV film:** stock OPC drums are tuned for red/NIR; the 405 nm SLA source wants a
  **UV/blue-responsive photoconductor** — amorphous selenium (a-Se; ~150:1 blue:red at
  10 µm), or wide-bandgap **ZnO / TiO₂**. UV is if anything cleaner (photon energy above
  bandgap).
- **Caveats:** proven for *polymer* (toner-like) powder; *metal* powder is much harder
  (won't hold triboelectric charge the same; sintering needs far more heat than a fuse
  roller). MSLA LCDs degrade under UV+heat and pass only ~5–8% UV (fuse away from the
  panel; exposure dose vs. photoconductor sensitivity is a calibration unknown).
  Layer-to-layer powder registration is electrophotographic AM's known hard problem.

## The principle that falls out

The architecture **rewards processes whose geometry is set by a digital pattern or a
deterministic path** — bitmap-per-layer (MSLA, electrophotography), toolpath (cold
spray, SPIF), or a bend program (wire, foil) — and **punishes stochastic thermal /
plasma parameter soup** (EDM, plasma spray, PVD), where the output is an empirical
function of a dozen coupled knobs. When a process is soup, either **consume it as a
black box** (buy the coating) or **don't use it** — don't pretend to software-define it.
That single test explains every verdict in the table above.

## Sources

- Electrophotography / xerography: <https://en.wikipedia.org/wiki/Electrophotography>
- Evolve Additive Solutions STEP: <https://www.asme.org/topics-resources/content/laser-printer-evolves-to-3d-printing>
- a-Se blue/UV photoresponse: <https://pmc.ncbi.nlm.nih.gov/articles/PMC3859090/>
- ZnO UV photodetectors: <https://pmc.ncbi.nlm.nih.gov/articles/PMC3231239/>
