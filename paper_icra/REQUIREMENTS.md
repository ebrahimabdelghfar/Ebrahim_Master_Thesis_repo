# ICRA 2027 — acceptance requirements, and how this version meets them

Source: [Call for ICRA 2027 Papers](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/) (fetched 2026-09-05).
Submission system: PaperPlaza (`ras.papercept.net`).

## Hard rules

| Rule | Value | Status here |
|---|---|---|
| Page limit | **8 pages**, complete paper: text, figures, tables, acknowledgment, references, appendix. No paid extra pages. | `main.pdf` is 8 pages; `build.sh` prints the count on every build |
| Over-length | Returned without review | — |
| Format | IEEE conference double column, US letter | `\documentclass[conference]{IEEEtran}`, 612×792 pt |
| Review | **Double anonymous**: no author names or affiliations in the PDF | ⚠ **The PDF currently carries the real author block.** `main.tex` keeps the anonymous block commented out directly beneath it — swap the two back before uploading. Everything else is already anonymous: no acknowledgment, funding, ethics or data-availability sections, and no author-owned repository URLs |
| Keywords | At least three, from the IEEE-RAS keyword list | `IEEEkeywords` in `sections/abstract.tex` |
| Dual submission | Not simultaneously under review elsewhere; an arXiv preprint is allowed; a workshop paper *with a DOI* disqualifies | Author decision |

## Dates

| Milestone | Date |
|---|---|
| Paper submission | **2026-09-15, 23:59 PST** |
| Video upload | 2026-08-05 → 2026-09-09, reopened 2026-09-17 → 2026-09-22 |
| Acceptance notification | 2027-01-31 |
| Final paper | 2027-02-06 |
| Author registration | 2027-03-05 |

## Video attachment (optional, strongly encouraged)

One file: mpeg/mp4/mpg, ≤ 20 MB, ≤ 180 s, minimum height 480 px, ≥ 20 fps, progressive scan. A CARLA closed-loop clip of the identified model driving the raceline is the obvious candidate; it is not produced yet.

## What this version contains, and what was dropped

The journal manuscript in `paper/` is 23 pages plus a 7-page supplement. There is no supplementary PDF at ICRA — everything except the video must fit in the 8 pages — so this version keeps the argument and the numbers and drops the depth:

- **Kept in full:** the reachable-friction bound and its correction, the identifiability sweep (Fig. 1, both axes plus the realised-demand collapse), the closed-loop comparison and the identified tire curves (Fig. 2), the admissibility gates, and every honest caveat — the prior's provenance, single run per configuration, the two axes on which the baseline wins.
- **Results are shown as graphs, not tables.** Both result tables of the journal version were replaced by the plots the same runs produced: the identifiability-sweep table by `figures/identifiability_sweep.pdf`, and the cross-run comparison table by a four-panel figure built from `graphs/comparison/` (tracking error by controller, peak-factor error, axle-force RMSE, identified tire curves). Every number those tables carried that supports a claim is now stated inline in the text.
- **Dropped:** the appendix and all supplementary material, the bridge-architecture and pipeline figures, the raceline figure, the algorithm listing, the vehicle-parameter table (its numbers are inline in Section IV), the per-configuration table (inline in Section VI-A), the μ time series, the one-step prediction bar chart (its numbers are inline), the S4D cost breakdown, and the baseline-versus-this-work axis table.
- **Compressed:** every section. Related work is one third of its journal length; the method keeps the derivations that carry the claim and states the rest once.
- **References:** 16, down from 38. Nothing that supports a claim in the text was removed; the citations dropped were for tools and background named in the prose.

No result, number, or claim was changed. The tire-curve plot is cropped to its two axle panels (the coefficient panel below them duplicated text that is now inline).

## Before submitting

1. Re-check the page count after any edit (`./build.sh` prints it).
2. Enter all co-authors in PaperPlaza even though the PDF is anonymous.
3. Pick ≥ 3 keywords from the RAS list in PaperPlaza; the `IEEEkeywords` block is a starting point, not the PaperPlaza selection.
4. Run the PaperCept PDF compliance test.
5. If a code link is wanted, use an anonymized mirror — the author's GitHub username must not appear.
6. Affiliation marker ‡ (author 2) has no affiliation line, exactly as in the journal manuscript; add it or drop the marker.
