#!/usr/bin/env bash
# Builds the three PDFs of the IJDC submission. Pass --figures to re-render
# Figures/ from paper_short first.
set -euo pipefail
cd "$(dirname "$0")"

[ "${1:-}" = "--figures" ] && tools/make_figures.sh

for doc in main supplementary; do
  pdflatex -interaction=nonstopmode "$doc" >/dev/null
  bibtex "$doc" >/dev/null
  pdflatex -interaction=nonstopmode "$doc" >/dev/null
  pdflatex -interaction=nonstopmode "$doc" >/dev/null
  echo "$doc.pdf: $(pdfinfo "$doc.pdf" | awk '/^Pages/{print $2}') pages"
done

pdflatex -interaction=nonstopmode sn-title-page >/dev/null
echo "sn-title-page.pdf"

# The manuscript and the supplement are what reviewers see, so nothing in them
# may name an author, an institution or a repository.
if pdftotext main.pdf - | grep -qiE 'abdelghfar|abuelanin|abdelaziz|elsayed|ain shams|autotronics|asurt|ASU_Bridge|github.com/ebrahim'; then
  echo "BLINDING LEAK in main.pdf" >&2; exit 1
fi
if pdftotext supplementary.pdf - | grep -qiE 'abdelghfar|abuelanin|abdelaziz|elsayed|ain shams|autotronics|asurt|ASU_Bridge|github.com/ebrahim'; then
  echo "BLINDING LEAK in supplementary.pdf" >&2; exit 1
fi
echo "blinding check: clean"
