#!/usr/bin/env python3
"""Renders paper_short's supplement into the accompanying sn-jnl document.

Shares the rewrite rules of assemble.py, and additionally renumbers the
pointers into the main article: the IEEEtran build numbered its sections
V-B and its tables in roman, and the Springer build numbers both in arabic.
The supplement's own floats and equations are prefixed S so that a reference
across the two documents is unambiguous.
"""
import re
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from assemble import SEC, SUB, common  # noqa: E402

# Section V-B of the IEEEtran build is Section 5.2 of this one. The two
# entries that are not a straight transliteration are corrections: the
# IEEEtran source pointed at V-F and V-G for facts that are in V-E and V-F.
MAIN = [
    ("Section~V-A1 of the main paper", "Section~5.1 of the main article"),
    ("Section~V-B of the main paper", "Section~5.2 of the main article"),
    ("Section~V-C of the main paper", "Section~5.3 of the main article"),
    ("Section~V-F of the main paper", "Section~5.5 of the main article"),
    ("Section~V-G of the main paper", "Section~5.6 of the main article"),
    ("Section~II of the main paper", "Section~2 of the main article"),
    ("Section~VI of the main paper", "Section~6 of the main article"),
    ("Table~I of the main paper", "Table~1 of the main article"),
    ("Table~III of the main paper's", "Table~3 of the main article's"),
    ("Fig.~2 of the main paper", "Figure~1 of the main article"),
    ("Equation~(3) of the main paper", "Eq.~(3) of the main article"),
    ("(3) of the main paper", "Eq.~(3) of the main article"),
    ("(5) of the main paper", "Eq.~(5) of the main article"),
    ("(1)--(2) of the main paper", "Eqs.~(1)--(2) of the main article"),
    ("main-paper (1)--(2)", "Eqs.~(1)--(2) of the main article"),
    ("main-paper (3)", "Eq.~(3) of the main article"),
    ("main-paper (4)", "Eq.~(4) of the main article"),
    ("the tire law (4)\nthere", "the tire law of Eq.~(4) of the main article"),
    ("the one\n(7) assumes", "the one Eq.~(7) of the main article assumes"),
    ("of the main paper", "of the main article"),
]

PREAMBLE = r"""%%=============================================================================
%% Supplementary information for
%% "Excitation-Aware On-Track System Identification at Full Scale"
%%
%% Submitted alongside the blinded manuscript and blinded on the same terms.
%% Its own sections, equations, figures and tables are numbered with an S
%% prefix, so a reference here and a reference in the main article can never
%% be confused.
%%
%% Build: pdflatex supplementary && bibtex supplementary && pdflatex
%%        supplementary && pdflatex supplementary
%%=============================================================================
\documentclass[12pt]{sn-jnl}

\usepackage{cite}%
\usepackage{graphicx}%
\usepackage{multirow}%
\usepackage{amsmath,amssymb,amsfonts}%
\usepackage{amsthm}%
\usepackage{mathrsfs}%
\usepackage{xcolor}%
\usepackage{textcomp}%
\usepackage{manyfoot}%
\usepackage{booktabs}%
\usepackage{array}%

%% See main.tex for why threeparttable is replaced rather than loaded.
\renewenvironment{threeparttable}{}{}
\renewenvironment{tablenotes}[1][]{%
  \par\vspace{2pt}\begingroup\raggedright\def\item{\par\noindent}\ignorespaces}%
  {\par\endgroup}

%% Minimal siunitx replacement; identical to the block in main.tex.
\makeatletter
\newif\ifsiu@prev
\DeclareRobustCommand{\siu@sep}{\ifsiu@prev\,\fi}
\DeclareRobustCommand{\siu@unit}[1]{\siu@sep\text{#1}\siu@prevtrue}
\DeclareRobustCommand{\siu@pre}[1]{\siu@sep\text{#1}\siu@prevfalse}
\DeclareRobustCommand{\siu@div}[1]{\text{#1}\siu@prevfalse}
\DeclareRobustCommand{\meter}{m}
\DeclareRobustCommand{\second}{s}
\DeclareRobustCommand{\gram}{g}
\DeclareRobustCommand{\newton}{N}
\DeclareRobustCommand{\radian}{rad}
\DeclareRobustCommand{\hertz}{Hz}
\DeclareRobustCommand{\percent}{\%}
\DeclareRobustCommand{\g}{\textit{g}}
\DeclareRobustCommand{\kilo}{k}
\DeclareRobustCommand{\milli}{m}
\DeclareRobustCommand{\per}{/}
\DeclareRobustCommand{\squared}{\ensuremath{^{2}}}
\providecommand{\degree}{\ensuremath{^{\circ}}}
\DeclareRobustCommand{\si}[1]{\begingroup
  \siu@prevfalse
  \renewcommand{\meter}{\siu@unit{m}}%
  \renewcommand{\second}{\siu@unit{s}}%
  \renewcommand{\gram}{\siu@unit{g}}%
  \renewcommand{\newton}{\siu@unit{N}}%
  \renewcommand{\radian}{\siu@unit{rad}}%
  \renewcommand{\hertz}{\siu@unit{Hz}}%
  \renewcommand{\percent}{\siu@unit{\%}}%
  \renewcommand{\degree}{\siu@unit{\ensuremath{^{\circ}}}}%
  \renewcommand{\g}{\siu@unit{\textit{g}}}%
  \renewcommand{\kilo}{\siu@pre{k}}%
  \renewcommand{\milli}{\siu@pre{m}}%
  \renewcommand{\per}{\siu@div{/}}%
  \renewcommand{\squared}{\ensuremath{^{2}}}%
  #1\endgroup}
\DeclareRobustCommand{\num}[1]{#1}
\DeclareRobustCommand{\numrange}[2]{#1\text{--}#2}
\DeclareRobustCommand{\SI}[2]{#1\,\si{#2}}
\DeclareRobustCommand{\SIrange}[3]{#1\text{--}#2\,\si{#3}}
\makeatother

\newcommand{\vecx}{\mathbf{x}}
\newcommand{\vecu}{\mathbf{u}}
\newcommand{\vecz}{\mathbf{z}}
\newcommand{\phip}{\Phi_{p}}
\newcommand{\Fyf}{F_{y,f}}
\newcommand{\Fyr}{F_{y,r}}
\newcommand{\bridge}{\textsc{SimBridge}}
\newcommand{\sysid}{\textsc{On-Track-SysID}}
\newcommand{\oursarm}{\texttt{Excitation-\allowbreak S4D-\allowbreak Pinn}}
\newcommand{\basearm}{\texttt{Baseline-\allowbreak NN-\allowbreak MSE}}

\renewcommand{\thesection}{S\arabic{section}}
\renewcommand{\theequation}{S\arabic{equation}}
\renewcommand{\thefigure}{S\arabic{figure}}
\renewcommand{\thetable}{S\arabic{table}}

\raggedbottom

\begin{document}

\title[Supplementary information]{Supplementary information for\\
Excitation-Aware On-Track System Identification at Full Scale:
A Simulation Study}

\abstract{This document holds material moved out of the main article: the
residual model's physics-augmented inputs and objective, the temporal residual
and the exact reduction of its cost, the per-phase identification timing, the
axis-by-axis comparison against the baseline pipeline as published, and the
supporting result figures. Nothing in the main article's argument depends on
it. Sections, equations, figures and tables are numbered with an S prefix here;
references of the form ``Section~5.2 of the main article'' point into the main
article instead.}

\maketitle
"""

BACKMATTER = r"""
\bibliographystyle{sn-nature}
\bibliography{references}

\end{document}
"""



def refuse_to_clobber(path):
    """main.tex and supplementary.tex carry edits these scripts cannot make.

    Layout fixes that only the compiled page shows -- a table rotated because
    it does not fit, a figure split because two panels do not -- live in the
    generated file, not here. Regenerating means redoing them, so the target
    has to be moved out of the way deliberately.
    """
    if path.exists():
        raise SystemExit(
            "%s already exists and is the source of record; move it aside to "
            "regenerate from paper_short." % path)


def main():
    body = common((SEC / "supplement.tex").read_text())
    # The source file opens with a note about resolving cross-references
    # through xr against main.aux, which this build does not do.
    body = body.split("\n", 4)[4]
    for old, new in MAIN:
        body = body.replace(old, new)
    leftover = re.findall(r"main paper|main-paper|Section~[IVX]+|Table~[IVX]+", body)
    if leftover:
        raise SystemExit("unconverted pointer into the main article: %s" % set(leftover))
    refuse_to_clobber(SUB / "supplementary.tex")
    (SUB / "supplementary.tex").write_text(PREAMBLE + "\n" + body + BACKMATTER)
    print("wrote", SUB / "supplementary.tex")


if __name__ == "__main__":
    main()
