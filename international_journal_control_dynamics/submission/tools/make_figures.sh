#!/usr/bin/env bash
# Renders the four TikZ diagrams of paper_short/figures into standalone PDFs
# under Figures/, and copies every raster/vector result figure the manuscript
# includes. The journal requires one .tex file and figures attached as separate
# files, so no TikZ source reaches the submitted manuscript.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
SUB="$(dirname "$HERE")"
REPO="$(cd "$SUB/../.." && pwd)"
SRC="$REPO/paper_short/figures"
OUT="$SUB/Figures"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

mkdir -p "$OUT"

preamble() {
  cat <<'EOF'
\documentclass[tikz,border=2pt]{standalone}
\usepackage{amsmath,amssymb}
\usepackage{tikz}
\usetikzlibrary{positioning,arrows.meta,fit,backgrounds,calc}
\newcommand{\bridge}{\textsc{SimBridge}}
\newcommand{\sysid}{\textsc{On-Track-SysID}}
\newcommand{\phip}{\Phi_{p}}
\newcommand{\vecx}{\mathbf{x}}
\newcommand{\vecu}{\mathbf{u}}
\newcommand{\vecz}{\mathbf{z}}
\newcommand{\Fyf}{F_{y,f}}
\newcommand{\Fyr}{F_{y,r}}
\newcommand{\SI}[2]{#1\,\si{#2}}
\newcommand{\num}[1]{#1}
\newcommand{\si}[1]{\text{#1}}
\newcommand{\meter}{m}\newcommand{\second}{s}\newcommand{\hertz}{Hz}
\newcommand{\radian}{rad}\newcommand{\percent}{\%}\newcommand{\newton}{N}
\newcommand{\kilo}{k}\newcommand{\milli}{m}\newcommand{\gram}{g}
\newcommand{\per}{/}\newcommand{\squared}{$^{2}$}
\begin{document}
EOF
}

for f in pipeline pipeline_s4 bridge_architecture frames; do
  # Two classes of edit, both because these diagrams now render on their own
  # rather than inside the manuscript. The bridge's real name and the CARLA
  # blueprint id name the institution, which a double-blind submission may not
  # do. The three \ref pointers have no .aux to resolve against here, so they
  # are frozen at the numbers main.tex and supplementary.tex print; re-check
  # them if a section, equation or table is added ahead of one.
  { preamble
    sed -e 's/\\textsc{Carla\\_ASU\\_Bridge}/\\bridge{}/g' \
        -e 's/{\\ttfamily asurt\\_fsai}/{\\ttfamily team\\_fsai}/g' \
        -e 's/\\eqref{eq:ssforces}/(7)/g' \
        -e 's/Sec.~\\ref{subsec:gates}/Sec.~7.1/g' \
        -e 's/Table~\\ref{sup:tabruntime}/Table~S1/g' "$SRC/$f.tex"
    echo '\end{document}'; } > "$WORK/$f.tex"
  ( cd "$WORK" && pdflatex -interaction=nonstopmode -halt-on-error "$f.tex" >/dev/null )
  cp "$WORK/$f.pdf" "$OUT/$f.pdf"
  echo "rendered $f.pdf"
done

cp "$SRC/raceline.pdf" "$OUT/raceline.pdf"

# The two scenario comparisons export identical basenames into different
# directories; flattening them into Figures/ needs the surface in the name.
for surface in constant decay; do
  d="$REPO/graphs/cmp_ours_vs_baseline_${surface}_mu"
  for fig in mu_timeseries_by_scenario control_timeseries_overlay_by_scenario \
             mu_error_by_scenario tire_force_rmse_by_scenario \
             tire_force_r2_by_scenario tire_force_error_hist_by_scenario \
             tire_force_timeseries_by_scenario state_rmse_by_scenario \
             state_error_hist_v_y state_error_hist_omega \
             state_timeseries_by_scenario e_y_error_hist_by_scenario \
             itae_by_scenario; do
    cp "$d/$fig.pdf" "$OUT/${fig}_${surface}_mu.pdf"
  done
done
echo "figures in $OUT"
