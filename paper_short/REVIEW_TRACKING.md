# paper_short — Simulated Peer Review and Response (2026-09-16)

This is the first standalone review pass run against `paper_short/` itself
(distinct from `paper/REVIEW_TRACKING.md`, which tracks the long paper's
lineage). It follows the same-day internal-consistency pass recorded in
`.wolf/buglog.json` bug-109 through bug-113, which this review treats as
already closed and does not re-litigate.

## 1. Reviewer Summary

| Title | Round | Verdict |
|---|---|---|
| Excitation-Aware On-Track System Identification at Full Scale: A Simulation Study | 1 | Minor Revision |

## 2. Initial Impression

The paper's argument is unusually disciplined: a closed-form bound
(`mu_reach`, Eq. 11) predicts a failure mode, the failure mode is verified on
a known tire (Table V), and every downstream number (Tables III, IV, VI–IX)
is cross-checked against that mechanism rather than merely reported. Nearly
every quantitative claim I attempted to re-derive from the tables printed
beside it (margins in the Introduction's contribution list, the beta sweep's
mean, the mu RMSE multipliers) checked out arithmetically. The paper is also
unusually candid about what it has not shown — see its own Limitations
(§VIII-B), which already cover the single-run and offline-ablation caveats a
reviewer would otherwise raise.

## 3. Criterion-Bound Dimension Judgements

| Dimension | Judgement | Evidence | Rationale |
|---|---|---|---|
| Originality | MEETS | Related Work §II-B explicitly separates this paper's claims from concurrent work (Wu et al.) and prior art (Hsu et al., Deep Dynamics), with a bounded "to our knowledge" novelty statement | The identifiability analysis of the virtual-data step is a clearly scoped, non-overlapping contribution |
| Methodological Rigor | PARTLY_MEETS | §VIII-B items 1 and 5 | Single run per configuration (no CI) and 4-of-8 offline (non-closed-loop) ablations are real gaps, but the paper discloses both without hedging the headline claim |
| Evidence Sufficiency | PARTLY_MEETS | Table I vs. Table II / §VI-D (see Issue 1) | One traceability gap found: two different front/rear axle-load splits appear under similar labels without a stated link |
| Argument Coherence | EXCEEDS | Introduction contributions 1-5 vs. §V-C/§V-D/§VI derivations | The chain from bound to degeneracy to correction to measured margin is explicit and internally cross-referenced throughout |
| Writing Quality | MEETS | Whole document | Dense but deliberately so (see `.wolf/cerebrum.md` 2026-09-16 AI-tell audit); no vocabulary padding found |

## 4. Strengths

- The reachable-friction bound (Eq. 11) is derived, then immediately used to explain a numeric pathology (Table V), then used again to fix the pipeline's operating point (§VI-B) — the same quantity does real work three times rather than being stated once and dropped.
- Nearly every headline number in the abstract and Introduction is a direct restatement of a table cell (verified: the 12.1×/10.5× force-RMSE ratios, the 9.0×/16.7× mu-RMSE ratios, and the beta-sweep mean of 23.7 N all reproduce from Tables IV, VI and the beta table to the reported precision).
- Limitations (§VIII-B) already anticipate the single-run and non-closed-loop-ablation objections a reviewer would otherwise raise as Major findings.

## 5. Issues by Severity

### 5.1 Critical
None found.

### 5.2 Major

| # | Section | Issue | Evidence | Suggested Fix | Effort |
|---|---|---|---|---|---|
| 1 | Table I (`tab:vehicle`) vs. Table II (`tab:ref`) / §VI-D | Table I reports a *measured* front/rear static split (51.2/48.8%). The per-axle loads used everywhere else in the paper, $F_{z,f}=1373$ N and $F_{z,r}=1274$ N, imply a 51.9/48.1% split instead — the one derived from $m,l_f,l_r$ via the static-transfer formula, not the measured one. Nothing connects the two splits, so a reader who checks $1373/(1373+1274)=51.9\%$ against Table I's 51.2% finds an unexplained 0.7-point gap. | Appendix A2 already reports "a 1.4% disagreement" between the measured and geometry-implied splits, in the context of $l_f,l_r$ — but does not point a reader from Table I or Table II to that reconciliation. | **Fixed in this round**: added footnote (b) to Table I's split row (`sections/platform.tex`), naming the 1373/1274 N values, their 51.9/48.1% split, and pointing to Appendix A2's existing 1.4% figure. No new computation; the 51.9/48.1% and the 1373/1274 N values were independently re-derived from $m, l_f, l_r$ via $F_{z,f/r}=mgl_{r/f}/L$ to confirm the fix states an existing fact rather than inventing one. | Quick Fix |

### 5.3 Minor

| # | Section | Issue | Evidence | Suggested Fix | Effort |
|---|---|---|---|---|---|
| 2 | Table I (`tab:vehicle`), row "Realised peak axle friction" | Reports a single value (0.71) where §VI-D later reports front and rear separately (0.71 front, 0.72 rear). A reader cannot tell from Table I alone whether 0.71 is the front value, an average, or a typo for the pair. | `sections/experiments.tex` §VI-D: "the force reaches $\max\lvert F_y\rvert/F_z=0.71$ front and $0.72$ rear" | Not changed in this round — recommend the author confirm against the underlying force-validation CSV (`graphs/identification/ours/...`) which value Table I's single number was drawn from before editing it, since I have not re-derived it myself and do not want to invent a number. | Quick Fix (pending author confirmation of source) |

### 5.4 Suggestions
None beyond the above.

## 6. Cross-Section Checks

| Check | Status | Notes |
|---|---|---|
| Title matches content | Pass | |
| Abstract reflects findings | Pass | Abstract's 3.2%/1.4%/11% figures and "no accepted coefficient rails" phrasing match §VI-D/§VI-E exactly |
| Introduction → Conclusion alignment | Pass | Five contributions in §I-C map one-to-one onto §V/§VI/§VII sections and are restated in the Conclusion |
| All tables/figures referenced in text | Pass | Spot-checked; no orphaned floats found |
| Word count / structure | Pass | Consistent with a "Simulation Study"-scoped IEEE T-IV submission |

## 7. Revision Instructions

Minor Revision: one Major item, addressed in this round by a one-footnote textual clarification (no data changed, no result renumbered). The Minor item is left to the author because resolving it requires reading a source CSV this review did not re-derive independently — flagging a number without verifying it against source data would itself be a fabrication risk this pipeline's rules (`shared/references/claim_strength_ladder.md`-equivalent discipline) exist to prevent.

## 8. Reviewer Confidence

Medium-High. The review is a single-agent simulated pass — not a substitute for a second human or external reviewer — and it inherits the same single-pass-no-independent-replication limitation the paper itself discloses about its own experiments. Every arithmetic cross-check reported above was recomputed from the numbers printed in the `.tex` sources during this review, not copied from the paper's own claims about itself.

---

## Response to Reviewers

**Major 1 (Table I / Table II axle-load split).** Addressed. `sections/platform.tex` Table I gained footnote (b) on the "Front/rear static split" row, stating that $F_{z,f}=1373$ N / $F_{z,r}=1274$ N (used in Table II and §VI-D) come from the geometry-implied 51.9/48.1% split via $m,l_f,l_r$, not from the measured 51.2/48.8% split reported in that row, and pointing to Appendix A2's existing 1.4% figure for the two splits' disagreement. No numeric result changes; this is a cross-reference addition only.

**Minor 2 (Table I peak-friction row).** Not addressed pending author confirmation. Recommend checking which of 0.71 (front) / 0.72 (rear) — or an average — Table I's single "Realised peak axle friction" value was drawn from, then either splitting the row into two values (matching §VI-D's presentation) or adding a footnote naming which axle/statistic it reports.
