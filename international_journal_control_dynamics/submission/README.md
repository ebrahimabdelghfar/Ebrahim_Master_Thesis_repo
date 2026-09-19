# IJDC submission

`paper_short/` ported to the Springer Nature `sn-jnl` class for the
*International Journal of Dynamics and Control*. Build everything with
`./build.sh`, or `./build.sh --figures` to re-render `Figures/` from
`paper_short/figures/` and `graphs/` first.

## What to upload

| File | Role |
| --- | --- |
| `main.tex`, `main.pdf` | The blinded manuscript. One `.tex` file, as the template requires. |
| `sn-title-page.tex`, `sn-title-page.pdf` | Authors, affiliations, acknowledgements, funding, contributions and the repository links. Submitted separately. |
| `supplementary.tex`, `supplementary.pdf` | Supplementary information, blinded on the same terms. |
| `Figures/*.pdf` | Every figure, attached separately. |
| `references.bib` | Shared bibliography. |
| `sn-jnl.cls`, `sn-nature.bst` | From the template package. See the note on `sn-nature.bst` below. |

## What changed against `paper_short/`

The journal's own instructions drove all of it.

- **One `.tex` document.** Every `\input` is inlined. The four TikZ diagrams
  are rendered to standalone PDFs by `tools/make_figures.sh` and included as
  image files, which also satisfies "all additional figures should be attached
  separately and not embedded in the TeX document itself".
- **Double-blind.** No author name, affiliation, repository URL or
  self-citation reaches `main.pdf` or `supplementary.pdf`; `build.sh` fails the
  build if one does. The bridge is called `SimBridge` there through one macro,
  and the CARLA blueprint id is blinded in the architecture diagram. The real
  names and links are on the title page, and `tools/make_figures.sh` holds the
  two substitutions to undo for the accepted version.
- **Abstract rewritten** to the template's guidance: under 200 words, no
  equations, no citations. The numbers it keeps are the ones the results
  section carries.
- **Citation style** follows the template's Required Citation Style section:
  `Figure \ref{}`, `Table \ref{}`, `Eq. \eqref{}`, `Eqs. \eqref{}`.
- **Single column.** `figure*` and `table*` have no meaning in `sn-jnl`, so
  every float is the plain variant and `\columnwidth` is `\textwidth`.
- **Layout the narrower text block forced.** The cross-run comparison table is
  rotated (`sidewaystable`); the pipeline, bridge-architecture, raceline and
  four supplement panel sets are rotated (`sidewaysfigure`) because upright
  they scale to two thirds or less and their labels stop being legible. The two
  two-panel result figures of Section 6 are split into one figure per surface,
  since the template asks authors to avoid subfigures.
- **Supplement cross-references renumbered.** `Section V-B` becomes
  `Section 5.2 of the main article`, `Table I` becomes `Table 1`, and so on.
  Two pointers in the `paper_short` source were off by one subsection and are
  corrected; `Fig. 2 of the main paper` pointed at the pipeline diagram, which
  is Figure 1. The supplement's own sections, equations, figures and tables
  carry an `S` prefix.

## Two workarounds

- **`threeparttable`.** `sn-jnl` loads it and then redefines `\tabular`, which
  leaves the package's saved `\tabular` defined in terms of itself: the second
  `threeparttable` in a document recurses until TeX runs out of grouping
  levels. Both environments are replaced in the preamble of `main.tex` and
  `supplementary.tex` by the plain block of note paragraphs the template asks
  for, so no table body needed editing.
- **`sn-nature.bst`.** The copy in the template package has lost the leading
  `booktitle` push from `format.in.ed.booktitle`, so every `@inproceedings`
  entry underflows BibTeX's literal stack and loses its venue. `sn-nature.bst`
  here carries the one-token fix, commented at the point of change;
  `sn-nature-as-shipped.bst` is the pristine copy. `references.bib` also gained
  an `address` field on eight conference entries, so it is now a copy that has
  diverged from `paper_short/references.bib`.

## Before submitting

Two statements in `sn-title-page.tex` are marked `CONFIRM BEFORE SUBMITTING`:
the affiliation of the second author, which the `paper_short` source marks with
a footnote it never defines, and the author-contribution statement, which no
source in this repository records. Springer also asks for a street and postcode
in each affiliation.

## Regenerating from `paper_short/`

`tools/assemble.py` and `tools/assemble_supplement.py` performed the mechanical
port and are kept as a record of exactly what it changed. They refuse to
overwrite `main.tex` and `supplementary.tex`, which are the source of record
now: the layout fixes above are in those files and not in the scripts.
