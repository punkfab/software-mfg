# software-defined manufacturing — run the sim, validate parts/cells/model.
#   make sim      open the SO-101 in the live interactive viewer (needs a display)
#   make render   headless scripted-motion video -> exports/renders/ (no display)
#   make check    run every validation gate
#   make help     list all targets
PY ?= python3
DISPLAY ?= :0

.PHONY: help sim view render check parts cells so101 clean
.DEFAULT_GOAL := help

sim: ## run the SO-101 in the live interactive viewer (needs a display; run via `!`)
	DISPLAY=$(DISPLAY) $(PY) -m mujoco.viewer --mjcf sim/so101/scene.xml

view: sim ## alias for `sim`

render: ## headless: render the scripted SO-101 motion -> exports/renders/ (no display)
	MUJOCO_GL=osmesa $(PY) scripts/so101_render.py

check: parts cells so101 ## run every validation gate (parts + cells + model)

parts: ## regenerate + validate local build123d parts -> exports/
	$(PY) scripts/check_parts.py

cells: ## pull geometry from external cells (../wirebender) -> exports/cells/
	$(PY) scripts/sync_cells.py

so101: ## validate the vendored SO-101 model (compile + position-control move)
	$(PY) scripts/so101_check.py

clean: ## remove generated artifacts under exports/ (keeps .gitkeep)
	find exports -type f ! -name .gitkeep -delete
	find exports -mindepth 1 -type d -empty -delete 2>/dev/null || true
	@echo "cleaned exports/"

help: ## list targets
	@grep -hE '^[a-z].*##' $(MAKEFILE_LIST) | sed 's/:.*## /\t/' | sort
