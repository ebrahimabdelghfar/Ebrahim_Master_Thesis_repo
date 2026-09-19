#!/usr/bin/env python3
"""Renders paper_short into the single-file Springer Nature manuscript IJDC asks for.

Inlines every \\input, rewrites the cross-reference wording to the template's
Required Citation Style, flattens the two-column floats to the single-column
ones sn-jnl sets, points the figure includes at the flattened Figures/
directory, and removes what would break the double-blind review.

Run from anywhere:  python3 tools/assemble.py
"""
import re
import pathlib

SUB = pathlib.Path(__file__).resolve().parent.parent
REPO = SUB.parent.parent
SEC = REPO / "paper_short" / "sections"

# --- cross-reference wording --------------------------------------------------


def citation_style(s):
    """Rewrites reference wording to the template's Required Citation Style."""
    s = re.sub(r"Figs?\.~\\ref", "Figure~\\\\ref", s)
    s = re.sub(r"Sec\.~\\ref", "Section~\\\\ref", s)
    s = s.replace("Equations~\\eqref", "Eqs.~\\eqref")
    s = s.replace("Equation~\\eqref", "Eq.~\\eqref")
    # Ranges and pairs take the plural form, and are matched before the
    # singular rule below can put an "Eq." in front of the first member.
    s = re.sub(r"(?<!Eqs\.~)\\eqref\{([^}]*)\}--\\eqref\{", r"Eqs.~\\eqref{\1}--\\eqref{", s)
    s = re.sub(r"(?<!Eqs\.~)\\eqref\{([^}]*)\} and~?\\eqref\{", r"Eqs.~\\eqref{\1} and \\eqref{", s)
    # Anything still bare. The lookbehind spares the members of the pairs above
    # and any \eqref this function has already prefixed.
    s = re.sub(r"(?<!Eq\.~)(?<!Eqs\.~)(?<!--)(?<!and )\\eqref\{", "Eq.~\\\\eqref{", s)
    return s


def single_column(s):
    """sn-jnl sets one column, so the spanning float variants have no meaning."""
    s = s.replace("\\begin{figure*}", "\\begin{figure}").replace("\\end{figure*}", "\\end{figure}")
    s = s.replace("\\begin{table*}", "\\begin{table}").replace("\\end{table*}", "\\end{table}")
    s = s.replace("\\columnwidth", "\\textwidth")
    return s


def figures(s):
    """Points every include at Figures/, where tools/make_figures.sh puts them."""
    s = s.replace("\\resizebox{0.98\\textwidth}{!}{\\input{figures/pipeline}}",
                  "\\includegraphics[width=\\textwidth]{Figures/pipeline.pdf}")
    s = s.replace("\\resizebox{0.98\\textwidth}{!}{\\input{figures/pipeline_s4}}",
                  "\\includegraphics[width=\\textwidth]{Figures/pipeline_s4.pdf}")
    s = s.replace("\\resizebox{\\textwidth}{!}{\\input{figures/bridge_architecture}}",
                  "\\includegraphics[width=\\textwidth]{Figures/bridge_architecture.pdf}")
    s = s.replace("\\resizebox{\\textwidth}{!}{\\input{figures/frames}}",
                  "\\includegraphics[width=\\textwidth]{Figures/frames.pdf}")
    s = s.replace("{raceline.pdf}", "{Figures/raceline.pdf}")
    s = re.sub(r"\{\\cmpstatic/([a-z0-9_]+)\}", r"{Figures/\1_constant_mu.pdf}", s)
    s = re.sub(r"\{\\cmpdecay/([a-z0-9_]+)\}", r"{Figures/\1_decay_mu.pdf}", s)
    # Supplement figures carry no directory macro; all of them are the
    # static-surface export.
    s = re.sub(r"\\includegraphics\[width=\\textwidth\]\{([a-z0-9_]+)\}(?!\.)",
               r"\\includegraphics[width=\\textwidth]{Figures/\1_constant_mu.pdf}", s)
    return s


def common(s):
    s = citation_style(s)
    s = single_column(s)
    s = figures(s)
    s = s.replace("\\IEEEPARstart{M}{odel-based}", "Model-based")
    return s


def read(name):
    return common((SEC / name).read_text())


# --- the manuscript -----------------------------------------------------------

PREAMBLE = r"""%%=============================================================================
%% Excitation-Aware On-Track System Identification at Full Scale
%%
%% International Journal of Dynamics and Control, Springer Nature sn-jnl class.
%% One .tex document, figures attached separately under Figures/, and blinded
%% for the journal's double-blind review: the author block, the affiliations
%% and the acknowledgements are in sn-title-page.tex instead.
%%
%% Build: pdflatex main && bibtex main && pdflatex main && pdflatex main
%%=============================================================================
\documentclass[lineno,12pt]{sn-jnl}

\usepackage{cite}%
\usepackage{graphicx}%
\usepackage{multirow}%
\usepackage{amsmath,amssymb,amsfonts}%
\usepackage{amsthm}%
\usepackage{mathrsfs}%
\usepackage[title]{appendix}%
\usepackage{xcolor}%
\usepackage{textcomp}%
\usepackage{manyfoot}%
\usepackage{booktabs}%
\usepackage{array}%
\usepackage{threeparttable}%

% --- lightweight algorithm float ---------------------------------------------
% float.sty ships with texlive-latex-recommended; the algorithm/algorithmicx
% families do not, so the body of Algorithm 1 is plain LaTeX rather than a new
% dependency.
\usepackage{float}%
\newfloat{algorithm}{tbp}{loa}
\floatname{algorithm}{Algorithm}
\newcommand{\algline}[2]{\makebox[1.8em][r]{\scriptsize#1:}\ \ #2\par}
\newcommand{\algin}[1]{\hspace*{#1em}}

% --- units -------------------------------------------------------------------
% Minimal siunitx replacement, implementing exactly the macros this manuscript
% uses. siunitx ships in texlive-science rather than texlive-latex-extra, so it
% cannot be assumed present; where it is, replacing this block with
% \usepackage{siunitx} plus \sisetup{detect-all, per-mode=symbol} and
% \DeclareSIUnit{\g}{\textit{g}} typesets the same units.
% All macros are robust: a table caption is a moving argument, and an
% expandable conditional breaks there.
\makeatletter
% Armed once a base unit has been emitted, disarmed after a prefix and after
% \per, so "kilo newton per radian" sets no space between k and N nor around
% the solidus.
\newif\ifsiu@prev
\DeclareRobustCommand{\siu@sep}{\ifsiu@prev\,\fi}
% \text{} so a unit typesets upright in both text and math mode: these macros
% are routinely used inside $...$, where a bare `m' would come out as an italic
% math variable.
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

\hyphenation{op-tical net-works semi-conduc-tor Pa-cej-ka iden-ti-fi-ca-tion}

% --- convenience macros -------------------------------------------------------
\newcommand{\vecx}{\mathbf{x}}
\newcommand{\vecu}{\mathbf{u}}
\newcommand{\vecz}{\mathbf{z}}
\newcommand{\phip}{\Phi_{p}}
\newcommand{\Fyf}{F_{y,f}}
\newcommand{\Fyr}{F_{y,r}}
% Blinded names. The simulator bridge and the identification package are named
% in sn-title-page.tex and in the unblinded release; here they carry names that
% do not identify the institution or the repository owner.
\newcommand{\bridge}{\textsc{SimBridge}}
\newcommand{\sysid}{\textsc{On-Track-SysID}}
% --- benchmark arm names -------------------------------------------------------
\newcommand{\oursarm}{\texttt{Excitation-\allowbreak S4D-\allowbreak Pinn}}
\newcommand{\basearm}{\texttt{Baseline-\allowbreak NN-\allowbreak MSE}}

\theoremstyle{thmstyleone}%
\newtheorem{theorem}{Theorem}%
\newtheorem{proposition}[theorem]{Proposition}%
\theoremstyle{thmstyletwo}%
\newtheorem{example}{Example}%
\newtheorem{remark}{Remark}%
\theoremstyle{thmstylethree}%
\newtheorem{definition}{Definition}%

\raggedbottom

\begin{document}

\title[Excitation-Aware On-Track System Identification at Full Scale]{Excitation-Aware On-Track System Identification at Full Scale: A Simulation Study}

%%=============================================================%%
%% Double-blind review: the author block, the affiliations, the
%% acknowledgements and the funding statement are in sn-title-page.tex.
%%=============================================================%%

\abstract{On-track system identification learns tire models from ordinary
racing laps instead of dedicated manoeuvres. We study one such
residual-learning pipeline at full vehicle scale, inside a simulator that
publishes per-wheel forces as ground truth. Transplanted unchanged from a 1:10
platform, it converges on a tire with less than half the vehicle's grip ceiling
and most Pacejka coefficients resting on their bounds. The cause is a loss of
excitation created by the pipeline's own virtual-data step: the friction the
synthetic rollout can demand is a property of the rollout configuration and not
of the tire, as a grid on a known tire confirms. Two further defects share that
source. The objective the coefficients are fitted against depends on them only
through one product, so the peak factor is not recoverable from the rollout at
any excitation, and the loop that feeds each fit its own previous answer is not
a contraction. We correct all three, and gate the controller interface on
derived physical quantities rather than on coefficient bounds. On a static
surface 33\% below the one the tire prior was measured on, axle cornering
stiffness is recovered to within 3.2\% and the peak factor to 11\%. On a
surface losing half its grip under the moving vehicle, the corrected pipeline
tracks the decay rate to 7\% and completes three timed laps at unchanged
lateral tracking error for 18\% of lap time, where the inherited configuration
holds a grip ceiling nearly four times the plant's and leaves the track.}

\keywords{System identification, Tire models, Vehicle dynamics, Autonomous
racing, Model predictive control, Persistent excitation}

\maketitle
"""

BACKMATTER = r"""
\bibliographystyle{sn-nature}
\bibliography{references}

%%=============================================================================
\begin{appendices}

"""

DECLARATIONS = r"""
\end{appendices}

\backmatter

\bmhead{Supplementary information}

This article has accompanying supplementary information, submitted as a
separate document (\texttt{supplementary.pdf}). It holds the residual model's
physics-augmented inputs and objective, the temporal residual and the exact
reduction of its cost, the per-phase identification timing, the axis-by-axis
comparison against the baseline pipeline as published, and the supporting
result figures. No argument in this article depends on it.

\section*{Declarations}

\begin{itemize}
\item \textbf{Data Availability}
Every figure and table in Section~\ref{sec:results} is exported together with
the CSV file holding its samples, and the cross-run comparison reads those CSV
files rather than re-deriving any metric. The scenario definitions, the
exported result CSV files, and the figure and sweep generators are released
with the source repositories named in Appendix~\ref{sec:refimpl}. The
repository links are withheld from this blinded manuscript and are on the
title page.

\item \textbf{Code Availability}
The identification package, the simulator bridge, and the model predictive
control and benchmarking packages are released as open source; see
Appendix~\ref{sec:refimpl}. The identification package extends the released
baseline of~\cite{ontrack2025} under the MIT licence that its package manifest
declares; the simulator bridge is GPL-3.0. The two run as separate ROS~2
overlays communicating only over ROS~2 topics and services, and neither links
against the other.

\item \textbf{Conflict of Interest}
The authors declare no competing interests relevant to the content of this
article.

\item \textbf{Ethics approval}
Not applicable. All results in this article are obtained in simulation. No
experiment involved human subjects, personal data, live vertebrates, or the
operation of a physical vehicle.

\item \textbf{Funding}
Not applicable. This work received no specific grant from any funding agency in
the public, commercial, or not-for-profit sectors.

\item \textbf{Author Contributions}
Stated on the separate title page, as the journal's double-blind procedure
requires.
\end{itemize}

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
    body = "\n".join(
        read(n) for n in ["intro.tex", "related_work.tex", "problem.tex", "platform.tex",
                          "method.tex", "experiments.tex", "integration.tex",
                          "discussion.tex", "conclusion.tex"]
    )
    appendix = read("appendix.tex")
    out = PREAMBLE + "\n" + body + BACKMATTER + appendix + DECLARATIONS
    refuse_to_clobber(SUB / "main.tex")
    (SUB / "main.tex").write_text(out)
    print("wrote", SUB / "main.tex")


if __name__ == "__main__":
    main()
