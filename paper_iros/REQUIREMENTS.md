# IROS 2027 — acceptance requirements, and how this version meets them

Sources: the [IEEE-RAS IROS 2027 call-for-papers entry](https://www.ieee-ras.org/event/call-for-papers-paper-submission-deadline-iros-2027-ieee-rsj-international-conference-on-intelligent-robots-and-systems-iros-27403-0/) and, for the rules the 2027 site has not published, the [IROS 2026 call for papers](https://2026.ieee-iros.org/contribute/call-for-papers/) (both fetched 2026-09-05).
IROS 2027: Florence, Italy, 2027-09-26 → 2027-10-01. Submission system: PaperPlaza.

**The official IROS 2027 call is not out yet.** Rows marked *(2026)* are carried over from IROS 2026 and must be re-checked before submitting.

## Hard rules

| Rule | Value | Status here |
|---|---|---|
| Page limit | **8 pages** including references and appendices *(2026)*; no paid extra pages | `main.pdf` is 8 pages |
| PDF size | ≤ 6 MB *(2026)* — tighter than ICRA | 0.3 MB; `build.sh` prints the size on every build |
| Format | IEEE conference double column, US letter | `\documentclass[conference]{IEEEtran}` |
| Review | **Double anonymous**, per the IEEE RAS rules | ⚠ **The PDF currently carries the real author block.** `main.tex` keeps the anonymous block commented out directly beneath it — swap the two back before uploading. Everything else is already anonymous |

## Dates

| Milestone | Date |
|---|---|
| Paper submission | **2027-03-01, 23:59 America/Los_Angeles** |
| Video upload | a few days after the paper deadline *(2026 pattern: +3 days)* |
| Acceptance notification | mid-June 2027 *(2026 pattern)* |
| Final manuscript | mid-July 2027 *(2026 pattern)* |

Roughly six months of runway. Every open experiment item in `.wolf/STATUS.md` — M1, M2, M7, M8, M9 and the seed fix — fits before this deadline, which is the argument for treating IROS as the primary target and ICRA as the stretch attempt.

## Video attachment (optional)

MPG/MPEG/MP4, ≤ 10 MB, ≤ 60 s *(2026)*: half the length and half the size ICRA allows, so cut the ICRA clip down rather than producing a second one.

## ICRA→IROS transfer track

IROS accepts papers rejected from the immediately preceding ICRA under an "ICRA-IROS transfer" category, with an extra author-response PDF answering the ICRA reviews. The ICRA 2027 decision lands 2027-01-31, a month before this deadline, so if that submission is rejected the transfer track is the direct path — keep the reviews and write the response rather than starting over.

## Contents

This is the same paper as `paper_icra/`, with the venue line in the title block changed; both are 8-page conference versions of the journal manuscript in `paper/`. See `paper_icra/REQUIREMENTS.md` for what was kept, dropped and compressed relative to the journal version.

## Before submitting

1. Re-check the 2027 call for the page limit, PDF size cap and video specification; update this file.
2. Re-check the page count and PDF size after any edit (`./build.sh` prints both).
3. Enter all co-authors in PaperPlaza even though the PDF is anonymous.
4. If the ICRA 2027 submission was rejected, submit under the ICRA-IROS transfer category with the author-response PDF.
