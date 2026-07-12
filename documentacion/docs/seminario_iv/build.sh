#!/bin/bash
# Compilación completa: pdflatex → bibtex → pdflatex × 2
DIR=/home/calafaker/Control-Formation/documentacion/docs/seminario_iv
cd "$DIR"
pdflatex -interaction=nonstopmode main.tex > /dev/null
bibtex main > /dev/null 2>&1 || true
pdflatex -interaction=nonstopmode main.tex > /dev/null
pdflatex -interaction=nonstopmode main.tex 2>&1 | grep -E "Output written|! LaTeX Error"
echo "PDF: $DIR/main.pdf"
