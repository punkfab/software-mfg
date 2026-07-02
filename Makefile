# software-defined manufacturing — run the sim, validate parts/cells/model.
#   make sim      open the SO-101 in the live interactive viewer (needs a display)
#   make render   headless scripted-motion video -> exports/renders/ (no display)
#   make check    run every validation gate
#   make help     list all targets
PY ?= python3
DISPLAY ?= :0

.PHONY: help sim printer-sim view render check parts cells so101 workcell-check workcell toolchange-check toolchange eject-check eject printer-cell bend bend-check cell-handoff cell-handoff-check press press-check opgraph opgraph-run opgraph-check pipeline pipeline-check calib calib-check freecad freecad-roundtrip clean
.DEFAULT_GOAL := help

sim: ## live viewer: the SO-101 ARM scene (needs a display; run via `!`)
	DISPLAY=$(DISPLAY) $(PY) -m mujoco.viewer --mjcf sim/so101/scene.xml

printer-sim: ## live viewer: the P1S PRINTER cell, looping the eject sequence (run via `!`)
	DISPLAY=$(DISPLAY) $(PY) scripts/printer_view.py

view: sim ## alias for `sim` (the arm)

render: ## headless: render the scripted SO-101 motion -> exports/renders/ (no display)
	MUJOCO_GL=osmesa $(PY) scripts/so101_render.py

check: parts cells so101 workcell-check toolchange-check eject-check bend-check cell-handoff-check press-check opgraph-check pipeline-check calib-check ## run every validation gate

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
