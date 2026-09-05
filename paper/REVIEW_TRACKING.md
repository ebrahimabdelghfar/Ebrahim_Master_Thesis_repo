# Review tracking — reviewer pass of 2026-09-05

One row per finding of `PAPER_REVIEW_FIXES_PROMPT.md`. Status is one of
**done** / **partial** / **blocked** / **open**. Build after this pass:
`paper/build.sh` → 27 pages, 0 errors, 0 overfull hboxes, 0 LaTeX warnings,
0 undefined references, 0 font warnings.

## Phase 1 — corrections that need no new run

| # | Status | Edit site | Evidence |
|---|---|---|---|
| F1.1 warm start is first-cycle-only | done | `sections/method.tex` §V-H | `On-Track-SysID/src/on_track_sys_id.py:420` (`if not self._is_first_identification ... return`) |
| F1.2 `delta_max` 0.34 → 0.54 rad | done | `sections/method.tex` §V-E | `params/pacejka_params.yaml: sweep_delta_max: 0.54`. Margin restated: 0.54 rad is 1.9× the 0.279 rad lock, the inherited 0.4 rad sweep 43 % above it |
| F1.3 window 0.4 s → 0.7 s | done | `sections/method.tex` §V-A3; `params/nn_params.yaml:49` comment | 20 × `sample_dt` 0.035 = 0.7 s. Re-justified as a configured value ≈30× the fastest lateral mode's time constant; the "matched to tire-relaxation timescales" claim is removed |
| F1.4 `T_s` provenance | done | `sections/platform.tex` §IV-E, `sections/appendix.tex` A8, `params/nn_params.yaml:3` | 0.035 matches neither 0.033 nor 1/50 = 0.020. Now stated as empirical, carried because identification at 0.020 and 0.035 returns the same accepted set. The wrong yaml comment ("MUST match 1/rate") is replaced |
| F1.5 separate the two timing mismatches | done | `sections/platform.tex` §IV-E | 50 Hz timer vs 0.033 s step = 65 % (carries the zero-order-hold artefact); 0.035 vs 0.033 = 6 % (the discretisation choice) |
| F2 practical vs structural identifiability | done | `sections/problem.tex` §III-C (retitled *Practical identifiability under limited excitation*), `sections/intro.tex`, `sections/related_work.tex` | `raue2009practical` and `astrom1995adaptive` added to `references.bib`; Bellman/Åström kept as the structural contrast only |
| F3 prior provenance | done | new §VI-A `\subsubsection{Provenance of the tire prior}` (`sections/experiments.tex`), plus abstract, §I contribution 6, conclusion | `params/pacejka_params.yaml` header (fit of `/sim/feedback/tire_forces` over an α = 0.37 rad, μ = 1.05, 1.50 g run); `graphs/comparison/comparison_summary.csv` `ours` row is the prior to float32. Claims changed from "identified" to "preserved" |
| F4 mass 269.8 → 269.6 kg | done | Table I (`platform.tex`), §I, §V-F, Table VII | 2644.5 / 9.81 = 269.6; `g` now stated in the Table I footnote. Unsprung row 29.8 → 29.6 kg; the 11 % μ bias is unchanged (29.6/269.6) |
| F4 contribution 2 drift per axle | done | `sections/intro.tex` | 1.009 → 1.5402 front, 1.002 → 1.8721 rear |
| F4 reference vs achieved speed range | done | `sections/intro.tex` | reference profile 9.6–25.8 m/s; identification data 9.2–26.5 m/s (§V-A2) |
| F4 gate 3 "100-stage horizon" | done | `sections/integration.tex` | `mpc_path_tracking/config/mpc_path_tracking.yaml:19` `N: 20`. The 2.3–6.8 ms figure had no artefact and is removed; replaced by the measured QP time from `graphs/control/*/speed_and_compute_cost.csv` (median 0.36 ms, p95 0.62 ms, max 3.81 ms) against the 50 ms control period |
| F4 20 Hz vs 50 Hz | done | `sections/method.tex` §V-H, `sections/appendix.tex` A8 | `on_track_sys_id.py:90` `self.rate = 50` is the identifier's acquisition/state timer; 20 Hz is the controller period |
| F4 `delta_sweep` / `delta_max` notation | done | `sections/intro.tex` §I-1 | now `\delta_{\mathrm{sweep}}` in the abstract, Eq. (14) and §I-1 alike |
| F4 abstract ≤ 250 words | done | `sections/abstract.tex` | 271 → 246 words, with the F3 softening included |
| F4 baseline understeer margin | done | `sections/experiments.tex` §VI-D | at F_z,f = 1373 N / F_z,r = 1274 N and the `comparison_summary.csv` coefficients: l_f C_f = 14.10 vs l_r C_r = 14.38 kN·m/rad → 2 %; `Ours` keeps 12 % |
| F4 Eq. (14) is an upper bound | done | `sections/method.tex` §V-D; Table V footnote | Ackermann κ = δ/L; true steady-state κ = δ/(L + K_us v²/g). Now measured: the sweep logs the realised max\|v_x ω\|/g, and at v = 20 m/s, δ_sweep = 0.4 the formula reads 10.64 against a realised 1.40–2.18 |
| F4 Table V CI pooling | done | `paper/figures/make_identifiability_sweep.py::summarise`, Table V | interval is now the 95 % t-interval over the five noise realisations (each contributing the mean of its own front/rear pair), n = 5, not over ten fits. Per-axle means and intervals are columns of `identifiability_sweep_summary.csv` |
| F4 soft boundary at 6 m/s | done | `sections/experiments.tex` §VI-B | the 1.05 tire returns 1.0394 at μ_reach = 0.96, i.e. 1 % at an excitation just below its own D |
| F4 Fig. 8 slip band | **blocked** | `pacejka_identified_vs_nominal_by_scenario` | regenerating it means running `compare_scenarios.py`, and `pgrep -f run_benchmark.py` matched (pid 17383) throughout this session. Not run — see the 2026-09-05 Do-Not-Repeat entry |
| F4 `fig2.png` dpi | done (by removal) | `sections/platform.tex` | 524 × 457 px ≈ 175 dpi at the printed width. It cannot be re-exported without a live simulator and carried nothing Tables I–II do not, so the figure is dropped |
| F4 `OT1/ptm/m/scit` font warning | done | `sections/platform.tex:185` | `\textsc` (via `\bridge`) inside the italic subsection heading; wrapped in `\textup`. `main.log` font warnings 1 → 0 |
| F5 `ahn2013friction` | done | `references.bib` | title corrected to "…Road **Frictional** Coefficient"; TCST 21(1):1–13, DOI 10.1109/TCST.2011.2170838 (Crossref) |
| F5 `hewing2020cautious` | done | `references.bib` | TCST 28(6):2736–2743, DOI 10.1109/TCST.2019.2949757 (Crossref) |
| F5 `hewing2018cautious` | done | `references.bib` | ECC 2018, pp. 1341–1348, DOI 10.23919/ECC.2018.8550162 (Crossref) |
| F5 `forzaeth2024` | done | `references.bib` | JFR 42(4):1037–1079, DOI 10.1002/rob.22429 (Crossref) |
| F5 `ontrack2025` | done (already complete) | `references.bib` | RA-L 10(2):3363–3370, DOI 10.1109/LRA.2025.3527336 |
| F5 arXiv IDs re-verified | done | — | 2603.09399 = "Vision-Augmented On-Track System Identification…"; 2602.07984 = "Analyzing the Impact of Simulation Fidelity…" (arXiv API, this session) |
| F5 missing prior work | done | `sections/related_work.tex` §II-A/B, `sections/method.tex` §V-F, `sections/problem.tex` §III-C | added `khaleghian2017survey`, `cabrera2004genetic`, `sierra2006cornering`, `astrom1995adaptive`, `raue2009practical`, each cited where the review placed it |
| F5 `physxvehicle` version | **partial** | `references.bib` | pinned to PhysX SDK 4.1 as bundled with UE 4.26 and linked by CARLA 0.9.16, with the matching NVIDIA-Omniverse `release/104.2` docs URL. **Needs author confirmation against the CARLA 0.9.16 build** — it is on the open-decisions list in `.wolf/STATUS.md` |
| F5 upstream licence named | done | `main.tex` Data-and-Code Availability | MIT, per `On-Track-SysID/package.xml:10` |
| F5 tagged release + Zenodo DOIs | **open** | `main.tex`, Appendix B | author action: mint the tag and the DOIs, then replace the branch URL |

## Phase 2 — runs

| # | Status | Notes |
|---|---|---|
| F6 repetitions and CIs | **open** | needs ≥ 5 seeds per arm via `benchmark_runner/scenarios.yaml`, after F11's seeds are set. Not run |
| F7 what the data contributes to D | **open** | needs the prior sweep (D of 0.6/1.0/1.5/2.0), a `prior_weight: 0.0` arm, and per-cycle realised μ_reach logging. Not run |
| F8 real baselines (NLS, EKF, inherited-as-published) | **open** | three new arms. Not run |
| F9 multi-step prediction at N = 20 | **open** | the benchmark node already computes 1/5/10-step metrics; they are not exported to `graphs/`. Not run |
| F10 brush warm start on the plant | **open** | needs per-cycle gate logging, the measured brush μ against 1.00–1.05, a mismatched-plant row for Table VIII, and the biased-μ̂ gate sweep. Not run |
| F11 reproducibility runs | **partial** | the `num_starts` note is written into §V-F (differential evolution ignores it, so the multi-start claim describes the baseline arm only). Still open: `pacejka_solver.seed` is `null` in both parameter sets, Table IX per-phase timers, and `I_z` from the CARLA actor inertia tensor — each needs a run |
| F12 strengthen §VI-B | **done** | see below |

### F12 in detail

`paper/figures/make_identifiability_sweep.py` now sweeps both arguments of
μ_reach and logs the demand the rollout actually realises. Re-run this session;
`identifiability_sweep.csv` (336 rows), `identifiability_sweep_summary.csv` and
`identifiability_sweep.pdf` are regenerated, and Table V now has two panels.

What the second axis showed is **not** what the review predicted, and §VI-B is
written from the artefact rather than from the expectation:

- Starving the sweep amplitude at a fixed v = 15 m/s destroys `D` just as
  starving the speed does, so speed is confirmed not to be the governing
  variable.
- But the two axes do not fail in the same direction. An under-speeded rollout
  returns too little grip (every tire back at D̂ ≈ 0.5); an under-swept one
  returns too much (the 1.05 tire returns 1.30 at δ_sweep = 0.06 rad, the 1.80
  tire rails on the 2.0 bound), with 2 of 8 coefficients on a box edge in every
  cell of panel (b) up to δ_sweep = 0.25 rad.
- They also do not collapse onto a common μ_reach, because μ_reach is the
  Ackermann upper bound: the realised max\|v_x ω\|/g saturates near the tire's
  peak while μ_reach keeps growing (10.64 against a realised 1.40–2.18 at
  v = 20 m/s). Against the realised demand the two axes do behave alike.
- §VI-B now says μ_reach is a screening bound — conclusive when it falls below
  D — and not a measure of achieved excitation, and records that with the
  residual network zeroed the model structure is exactly matched, so the real
  pipeline's degradation is at least this bad.

## Phase 3 — structure and venue

| # | Status | Notes |
|---|---|---|
| F13 cut to ≤ 14 pages, 3–4 contributions | **open** | 27 pages, six contributions. Merging 2 and 3, demoting 4 and the S4D cost half of 5, and moving §V-A3–A4, Table VII and Table IX to supplementary are author decisions; `.wolf/STATUS.md` already lists the contribution merge as open |
| F14 title | **done** | now *Excitation-Aware On-Track System Identification at Full Scale: A Simulation Study* |
| F14 keywords | **done** | redrawn from the IEEE taxonomy: autonomous vehicles, system identification, parameter estimation, vehicle dynamics, tires, predictive control, road vehicles, simulation |
