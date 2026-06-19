# software-defined manufacturing — run the sim, validate parts/cells/model.
#   make sim      open the SO-101 in the live interactive viewer (needs a display)
#   make render   headless scripted-motion video -> exports/renders/ (no display)
#   make check    run every validation gate
#   make help     list all targets
PY ?= python3
DISPLAY ?= :0

.PHONY: help sim view render check parts cells so101 workcell-check workcell toolchange-check toolchange clean
.DEFAULT_GOAL := help

sim: ## run the SO-101 in the live interactive viewer (needs a display; run via `!`)
	DISPLAY=$(DISPLAY) $(PY) -m mujoco.viewer --mjcf sim/so101/scene.xml

view: sim ## alias for `sim`

render: ## headless: render the scripted SO-101 motion -> exports/renders/ (no display)
	MUJOCO_GL=osmesa $(PY) scripts/so101_render.py

check: parts cells so101 workcell-check toolchange-check ## run every validation gate

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

clean: ## remove generated artifacts under exports/ (keeps .gitkeep)
	find exports -type f ! -name .gitkeep -delete
	find exports -mindepth 1 -type d -empty -delete 2>/dev/null || true
	@echo "cleaned exports/"

help: ## list targets
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
