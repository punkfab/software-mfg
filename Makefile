# software-defined manufacturing — run the sim, validate parts/cells/model.
#   make sim      open the SO-101 in the live interactive viewer (needs a display)
#   make render   headless scripted-motion video -> exports/renders/ (no display)
#   make check    run every validation gate
#   make help     list all targets
PY ?= python3
DISPLAY ?= :0

.PHONY: help sim printer-sim view render check parts cells so101 workcell-check workcell toolchange-check toolchange eject-check eject printer-cell bend bend-check cell-handoff cell-handoff-check press press-check opgraph opgraph-run opgraph-check pipeline pipeline-check calib calib-check foil-former foil-former-check foil-lom foil-lom-check glue glue-check coord coord-check assemble assemble-check interference interference-check layout layout-check mobile-base mobile-base-check mobile-base-mj mobile-base-mj-check feetech feetech-check scanning scanning-check omni omni-check coupling coupling-check camlock camlock-check camlock-preload camlock-preload-check lekiwi lekiwi-demo lekiwi-check tracking-check bridge bridge-check ir-solid ir-solid-check step-recognize step-recognize-check freecad freecad-roundtrip clean
.DEFAULT_GOAL := help

sim: ## live viewer: the SO-101 ARM scene (needs a display; run via `!`)
	DISPLAY=$(DISPLAY) $(PY) -m mujoco.viewer --mjcf sim/so101/scene.xml

printer-sim: ## live viewer: the P1S PRINTER cell, looping the eject sequence (run via `!`)
	DISPLAY=$(DISPLAY) $(PY) scripts/printer_view.py

view: sim ## alias for `sim` (the arm)

render: ## headless: render the scripted SO-101 motion -> exports/renders/ (no display)
	MUJOCO_GL=osmesa $(PY) scripts/so101_render.py

check: parts cells so101 workcell-check toolchange-check eject-check bend-check cell-handoff-check press-check opgraph-check pipeline-check calib-check foil-former-check foil-lom-check glue-check coord-check tracking-check interference-check layout-check mobile-base-check mobile-base-mj-check feetech-check scanning-check omni-check coupling-check camlock-check camlock-preload-check ir-solid-check step-recognize-check assemble-check bridge-check ## run every validation gate

parts: ## regenerate + validate local build123d parts -> exports/
	$(PY) scripts/check_parts.py

cells: ## pull geometry from external cells (../wirebender) -> exports/cells/
	$(PY) scripts/sync_cells.py

so101: ## validate the vendored SO-101 model (compile + position-control move)
	$(PY) scripts/so101_check.py

workcell-check: ## validate the workcell composes + IK reaches the work datum
	$(PY) scripts/workcell_check.py

workcell: ## render a preview still of the workcell -> exports/renders/workcell.png
	MUJOCO_GL=osmesa $(PY) sim/workcell.py

toolchange-check: ## validate the tool changer (couple -> carry -> present -> shear -> return)
	$(PY) scripts/toolchange_check.py

toolchange: ## render the full tool-change + shear sequence -> exports/renders/toolchange.mp4
	MUJOCO_GL=osmesa $(PY) scripts/toolchange_demo.py

eject-check: ## validate P1S eject-in-place removes the part into the bin
	$(PY) scripts/eject_check.py

eject: ## render the P1S eject-in-place sequence -> exports/renders/eject.mp4
	MUJOCO_GL=osmesa $(PY) scripts/eject_demo.py

printer-cell: ## render a preview still of the printer cell -> exports/renders/printer_cell.png
	MUJOCO_GL=osmesa $(PY) sim/printer_cell.py

bend: ## render the wire the bender produces (composed by reference) -> exports/renders/bent_wire.png
	MUJOCO_GL=osmesa $(PY) scripts/bend_render.py

bend-check: ## validate bend_wire runs the bender's real forward model (read-only)
	$(PY) scripts/bend_check.py

cell-handoff: ## render the bender->arm handoff: real bent wire presented + sheared
	MUJOCO_GL=osmesa $(PY) scripts/cell_handoff_demo.py

cell-handoff-check: ## validate the bender's real wire lands under the shear in the workcell
	$(PY) scripts/cell_handoff_check.py

press: ## render the C-frame bearing/insert press -> exports/renders/press.mp4
	MUJOCO_GL=osmesa $(PY) scripts/press_demo.py

press-check: ## validate the press seats the bearing AND is self-reacting (arm feels ~none)
	$(PY) scripts/press_check.py

opgraph: ## schedule the multi-cell job; print the Gantt + cycle time
	$(PY) scripts/opgraph_run.py

opgraph-run: ## schedule AND execute the real sim actions in order
	MUJOCO_GL=osmesa $(PY) scripts/opgraph_run.py --execute

opgraph-check: ## validate the schedule (precedence, resource limits, overlap win)
	$(PY) scripts/opgraph_check.py

pipeline: ## multi-unit pipeline: per-unit cycle time + scaling with a 2nd printer
	$(PY) scripts/pipeline_run.py

pipeline-check: ## validate pipelining amortizes cycle time + parallel machines scale
	$(PY) scripts/pipeline_check.py

calib: ## walk the calibration round-trip: query -> staleness stamp -> ingest a measurement -> diff
	$(PY) scripts/calib_demo.py

calib-check: ## validate the press sim runs on FRESH, in-envelope calibrated params (with margin)
	$(PY) scripts/calib_check.py

foil-former: ## render the foil former folding flat stock into a stiff profile -> exports/renders/
	MUJOCO_GL=osmesa $(PY) scripts/foil_former_demo.py

foil-former-check: ## validate the CNC foil bender's forward model (springback + work-hardening)
	$(PY) scripts/foil_former_check.py

foil-lom: ## plan a foil-LOM build: slice a solid into foil layers + print the per-layer op-graph
	$(PY) sim/foil_lom.py

foil-lom-check: ## validate the foil-LOM slice plan is manufacturable + schedulable
	$(PY) scripts/foil_lom_check.py

glue: ## render the switchable hot-glue tool (body + nozzle bracket) -> exports/renders/
	MUJOCO_GL=osmesa $(PY) scripts/glue_demo.py

glue-check: ## validate the hot-glue tool mounts the switcher + its forward model is sound
	$(PY) scripts/glue_check.py

coord: ## schedule the two-arm glue-and-hold task; print the coordination Gantt
	$(PY) orchestration/coord_job.py

coord-check: ## validate the two-arm coordination (hold never drops the part; needs 2 arms)
	$(PY) scripts/coord_check.py

assemble: ## run the end-to-end assembly: CAD -> plan -> CAM toolpath -> motion -> track -> verify
	$(PY) orchestration/assemble.py

assemble-check: ## validate the end-to-end assembly pipeline (reachable toolpath + verified placement)
	$(PY) scripts/assemble_check.py

interference: ## demo solid interference checking (static overlap + swept tool-vs-part)
	$(PY) scripts/interference_check.py

interference-check: ## validate solid part interference checking (catches collisions, fails safe on open meshes)
	$(PY) scripts/interference_check.py

layout: ## demo the material-handling floor (stations, routing, staging into the work envelope)
	$(PY) scripts/layout_check.py

layout-check: ## validate collision-checked station moves + material routed into the work envelope
	$(PY) scripts/layout_check.py

mobile-base: ## show the mobile-base design comparison (hold force, tip-over, motor sizing)
	$(PY) sim/mobile_base.py

mobile-base-check: ## validate the mobile-base physics + that a buildable base clears the printer-pick
	$(PY) scripts/mobile_base_check.py

mobile-base-mj: ## MuJoCo dynamic tip-over: arm reaches out until the base tips (vs analytic)
	$(PY) sim/mobile_base_mj.py

mobile-base-mj-check: ## validate the dynamic sim tips where the analytic statics predict
	$(PY) scripts/mobile_base_mj_check.py

feetech: ## demo the DIY DC-motor STS servo (codec + virtual servo + bus)
	$(PY) sim/feetech_protocol.py

feetech-check: ## validate a DC motor can speak the Feetech STS protocol as a LeRobot drop-in
	$(PY) scripts/feetech_protocol_check.py

scanning: ## demo 3D scanning: scan -> register to CAD -> recover pose + as-built deviation
	$(PY) sim/scanning.py

scanning-check: ## validate scan->CAD pose recovery (multi-view) + as-built deviation + re-anchor
	$(PY) scripts/scanning_check.py

omni: ## build the omni wheel STLs (roller + hub) + the assembly -> build/omni_*.stl
	$(PY) parts/omni_roller.py
	$(PY) parts/omni_hub.py
	$(PY) scripts/omni_demo.py

omni-check: ## validate the reverse-engineered omni wheel (assembles, rollers spin free, OD=R_EFF)
	$(PY) scripts/omni_check.py

coupling: ## build the tool-changer coupling faces + show the hold/registration statics
	$(PY) parts/coupling_arm_side.py
	$(PY) parts/coupling_tool_side.py
	$(PY) sim/coupling_statics.py

coupling-check: ## validate the coupling (keyed 1-orientation, prints clean, secures the tool)
	$(PY) scripts/coupling_check.py

camlock: ## build the servo draw-lock changer (tool + cam ring + arm) + show the statics
	$(PY) parts/camlock_tool_side.py
	$(PY) parts/camlock_cam_ring.py
	$(PY) parts/camlock_arm_side.py
	$(PY) sim/camlock_statics.py

camlock-check: ## validate the draw-lock (self-locks, no tool spin, seats stiffer than Fidlock)
	$(PY) scripts/camlock_check.py

camlock-preload: ## MuJoCo preload-maintenance study: creep retention + vibration -> build/camlock_preload.png
	$(PY) sim/camlock_preload_sim.py --demo

camlock-preload-check: ## validate the preload physics (rigid loses clamp on creep; a spring holds it)
	$(PY) sim/camlock_preload_sim.py --selftest

lekiwi: ## INTERACTIVE viewer: the exact LeKiwi (3-omni base + SO-101), composed by reference
	$(PY) sim/lekiwi_sim.py

lekiwi-demo: ## headless: LeKiwi drives holonomically + arm reaches -> build/lekiwi/lekiwi.gif
	$(PY) sim/lekiwi_sim.py --demo

lekiwi-check: ## validate the exact LeKiwi sim (loads, stands, drives holonomically, upright)
	$(PY) sim/lekiwi_sim.py --selftest

tracking-check: ## validate CAD-referenced pose tracking (staleness + verify vs nominal)
	$(PY) scripts/tracking_check.py

bridge: ## show which motion backend the so101-lab bridge resolves (Placo vs built-in)
	$(PY) -c "import bridge; print(bridge.status())"

bridge-check: ## validate the so101-lab bridge (Placo by reference, built-in fallback)
	$(PY) scripts/bridge_check.py

ir-solid: ## one IR -> a build123d SOLID via the featuretree b3d backend -> exports/freecad/*.b3d.stl
	$(PY) scripts/ir_solid.py coupling_plate

ir-solid-check: ## validate one IR renders a watertight build123d solid (volume-anchored to FreeCAD)
	$(PY) scripts/ir_solid.py --check

step-recognize: ## recover an IR from a STEP part (2.5D-prismatic), verified by re-emit
	$(PY) scripts/step_recognize.py

step-recognize-check: ## validate STEP->IR recognition (prismatic parts verify; out-of-scope flagged)
	$(PY) scripts/step_recognize.py --check

freecad: ## emit a FreeCAD .FCStd feature tree from the IR (needs the FreeCAD AppImage)
	$(PY) scripts/freecad_gen.py

freecad-roundtrip: ## IR -> FreeCAD tree -> human edit -> back into the IR -> regenerate
	$(PY) scripts/freecad_roundtrip.py

clean: ## remove generated artifacts under exports/ (keeps .gitkeep)
	find exports -type f ! -name .gitkeep -delete
	find exports -mindepth 1 -type d -empty -delete 2>/dev/null || true
	@echo "cleaned exports/"

help: ## list targets
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
