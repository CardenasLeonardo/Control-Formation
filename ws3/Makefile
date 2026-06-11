WS_DIR := $(shell pwd)
DOCS_IEEE := docs/congreso_template

# ── Experimentos ────────────────────────────────────────────────
# Uso: make exp NAME=consenso_lider
#      make exp NAME=consenso_prom LAUNCH=consenso_prom.launch.py
#      make exp NAME=consenso_lider LAUNCH=consenso_lider.launch.py ARGS="n_robots:=5"
NAME   ?= experimento
LAUNCH ?= consenso_lider.launch.py
ARGS   ?=

.PHONY: exp build paper install-latex last

exp: build
	@./run_experiment.sh $(NAME) $(LAUNCH) $(ARGS)

# ── Build ROS2 ───────────────────────────────────────────────────
build:
	@bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select control_nodes simulador 2>&1 | tail -5"

# ── Documentos LaTeX ─────────────────────────────────────────────
paper:
	@cd $(DOCS_IEEE) && pdflatex -interaction=nonstopmode main.tex > /dev/null 2>&1; \
	 bibtex main > /dev/null 2>&1; \
	 pdflatex -interaction=nonstopmode main.tex > /dev/null 2>&1; \
	 pdflatex -interaction=nonstopmode main.tex 2>&1 | grep -E "pages|Error" | head -5
	@echo "PDF generado: $(DOCS_IEEE)/main.pdf"

paper-clean:
	@cd $(DOCS_IEEE) && rm -f *.aux *.bbl *.blg *.log *.out *.toc *.fls *.fdb_latexmk

# ── Copiar figuras del último experimento al paper ───────────────
# Uso: make use-exp EXP=2026-05-07_15-30-00_consenso_lider
EXP ?=
use-exp:
	@if [ -z "$(EXP)" ]; then \
		echo "Error: especifica EXP=<nombre_del_directorio>"; \
		echo "Experimentos disponibles:"; \
		ls experiments/; \
	else \
		cp experiments/$(EXP)/figures/* $(DOCS_IEEE)/figures/ && \
		echo "Figuras copiadas de experiments/$(EXP)/figures/ → $(DOCS_IEEE)/figures/"; \
	fi

# Atajo: copia las figuras del experimento más reciente
last:
	$(eval LATEST := $(shell ls -td experiments/*/  2>/dev/null | head -1 | xargs basename))
	@if [ -z "$(LATEST)" ]; then \
		echo "No hay experimentos en experiments/"; \
	else \
		cp experiments/$(LATEST)/figures/* $(DOCS_IEEE)/figures/ && \
		echo "Figuras del más reciente ($(LATEST)) → $(DOCS_IEEE)/figures/"; \
	fi

# ── Ayuda ────────────────────────────────────────────────────────
help:
	@echo ""
	@echo "  make exp   NAME=<nombre> [LAUNCH=<launch>] [ARGS=<ros_args>]"
	@echo "             Corre experimento y guarda figuras automáticamente"
	@echo ""
	@echo "  make last  Copia figuras del experimento más reciente al paper"
	@echo "  make paper Compila docs/paper_ieee/main.tex → PDF"
	@echo "  make build Recompila paquetes ROS2"
	@echo ""
	@echo "  Flujo típico:"
	@echo "    make exp NAME=consenso_lider"
	@echo "    make last"
	@echo "    make paper"
	@echo ""