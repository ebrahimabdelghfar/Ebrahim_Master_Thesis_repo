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

# Supplementary material (moved-out sections). Independent document; it quotes
# the main paper's section numbers as text, so rebuild it after main.tex.
pdflatex -interaction=nonstopmode -halt-on-error supplement_main.tex
bibtex supplement_main
pdflatex -interaction=nonstopmode -halt-on-error supplement_main.tex
pdflatex -interaction=nonstopmode -halt-on-error supplement_main.tex
echo
echo "built supplement_main.pdf ($(pdfinfo supplement_main.pdf | awk '/^Pages/{print $2}') pages)"
grep -cE '^! ' supplement_main.log | xargs -I{} echo "  errors: {}"
grep -cE 'Overfull \\hbox' supplement_main.log | xargs -I{} echo "  overfull hboxes: {}"
grep -cE 'LaTeX Warning' supplement_main.log | xargs -I{} echo "  latex warnings: {}"
