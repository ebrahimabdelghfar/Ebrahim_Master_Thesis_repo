#!/usr/bin/env bash
# Build the paper. Requires: texlive-latex-extra (IEEEtran deps).
# Optional but recommended: texlive-science, which provides siunitx --
# without it main.tex falls back to units_fallback.tex (see its header).
set -e
cd "$(dirname "$0")"
pdflatex -interaction=nonstopmode -halt-on-error main.tex
bibtex main
pdflatex -interaction=nonstopmode -halt-on-error main.tex
pdflatex -interaction=nonstopmode -halt-on-error main.tex
echo
echo "built main.pdf ($(pdfinfo main.pdf | awk '/^Pages/{print $2}') pages)"
grep -cE '^! ' main.log | xargs -I{} echo "errors: {}"
grep -cE 'Overfull \\hbox' main.log | xargs -I{} echo "overfull hboxes: {}"
grep -cE 'LaTeX Warning' main.log | xargs -I{} echo "latex warnings: {}"
