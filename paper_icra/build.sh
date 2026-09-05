#!/usr/bin/env bash
# Build the ICRA conference version. Requires: texlive-latex-extra.
# Optional: texlive-science (siunitx); without it main.tex falls back to
# units_fallback.tex.
set -e
cd "$(dirname "$0")"
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
echo
echo "built main.pdf ($(pdfinfo main.pdf | awk '/^Pages/{print $2}') pages) -- ICRA limit is 8"
grep -cE '^! ' main.log | xargs -I{} echo "errors: {}"
grep -cE 'Overfull \\hbox' main.log | xargs -I{} echo "overfull hboxes: {}"
