# featuretree/ — operation-preserving CAD round-trip

The thing no neutral *file* format gives you: a parametric feature tree that
survives interchange **and** human editing. The approach — since the tree always
lives with its interpreter — is a neutral **feature IR (our DSL)** plus per-target
**emitters** that re-author it in each tool's native feature vocabulary.

```
            ┌───────────────────────────── ir.py (the DSL, source of truth) ──────┐
            │  named features · named params · symbolic geometry refs (no edge ids) │
            └──────────────┬───────────────────────────────┬──────────────────────┘
            emit ▼                                          ▼ emit
   FreeCAD .FCStd (native tree)                 [Onshape FeatureScript · Fusion API ·
   fc_build.py  ──────────────►  human edits      SolidWorks macro]  (roadmap)
   fc_read.py   ◄──────────────  (by feature name → IR, via update_from_freecad)
```

## Why round-trip works here (and not for neutral feature files)

- Every feature's IR `name` becomes the target object's **Label**, so a human
  edit is matched back **by name**, not by unstable kernel edge ids.
- Geometry is referenced **symbolically** (feature name + plane), the wall that
  kills portable feature files (topological naming) — sidestepped, not solved.

## Run it

```bash
make freecad            # IR -> FreeCAD .FCStd; open it, the tree is on the left
make freecad-roundtrip  # IR -> tree -> simulated human edit -> back into the IR -> regenerate
```

Needs FreeCAD (the `/opt` AppImage; override with `FREECAD_APPIMAGE` / `FREECAD_CMD`).
Driven through FreeCAD's *own* Python 3.11 via `freecadcmd`, same as other cells.
Cross-checked: the sample plate is 11497.3 mm³ in **both** FreeCAD and build123d
(`parts/example_plate.py`) — same DSL, two kernels, agreeing geometry.

## Scope and the honest road ahead

- **Working now:** XY sketches (rect + circle), Pad, Pocket (through/depth),
  **Fillet via edge query**, **face-attached sketches via face query**; parameter
  round-trip (lengths, radii, fillet radius) by name; FreeCAD backend. The real
  `coupling_plate` (Ø50×6 disc, bore, 3 mounts, filleted rim, top-face counterbore)
  is expressed as IR and round-trips — watertight, dimensions exact, volume agrees
  with build123d. `IR.SAMPLES = {plate, coupling_plate}`.
- **The topological-naming sidestep, concretely:** fillets/face-sketches store a
  *query* (`{"circles": "top_outer"}`, `{"face_of": "drill", "side": "top"}`), and
  the emitter re-resolves it to live `EdgeN`/`FaceN` every build — so edits never
  go stale. Next selectors: by-radius, by-position, by-count.
- **Next:** a build123d emitter (one DSL → both the sim/manufacturing path and the
  human-editable path); then Onshape FeatureScript and a SolidWorks macro emitter.
- **Won't pretend:** a SolidWorks `.SLDPRT` can't be written on Linux — that
  backend emits a macro to run on a SolidWorks seat.
