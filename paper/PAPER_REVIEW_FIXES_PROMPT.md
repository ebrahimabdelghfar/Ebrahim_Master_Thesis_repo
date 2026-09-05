# Paper revision prompt — reviewer pass of 2026-09-05

You are revising `paper/main.tex` and its sections in response to an expert review of
*Excitation-Aware On-Track System Identification at Full Scale* (IEEE T-IV target,
currently 26 pages, `paper/build.sh` clean).

Work through the findings below in the given order. Each has an **Evidence** line (where
the problem is), a **Do** line (what to change or run), and an **Accept** line (how to know
it is finished). Record every finding's status in `paper/REVIEW_TRACKING.md`, one row per
finding, with the edit site and the artefact the new number came from.

## Ground rules

- Every number in the paper must trace to an exported artefact under `graphs/` or to a
  config file in the repo. If a number cannot be traced, it is removed, not adjusted.
- The user starts and stops the CARLA simulator manually
  (`cd ~/Carla_ASU_Bridge && make launch_carla_sim AUTO_START=false`). Do not launch or
  kill the simulator. Drive only the bridge's lifecycle.
- Never run `compare_scenarios.py` while `pgrep -f run_benchmark.py` matches.
- The paper documents the physics and the rationale of the shipped solution. It does not
  narrate the development history. An empirical property of the estimator measured against
  the plant (e.g. an estimator returning a biased `mu` on this vehicle) is a result and
  belongs in the paper; a description of a coding mistake does not.
- After each run, regenerate `graphs/comparison/` and re-derive the affected §VI numbers
  from the CSVs rather than editing them by hand.
- Keep `paper/build.sh` at 0 errors, 0 overfull hboxes, 0 LaTeX warnings, 0 undefined
  references.

---

## Phase 1 — Corrections that need no new run

These are the cheapest and they block nothing. Do them first.

### F1. Paper contradicts the shipped configuration in four places

**Evidence**

| Paper says | Code says | Site |
|---|---|---|
| The friction warm start is recomputed on *every* cycle rather than once at startup | `if not self._is_first_identification ... return` — first cycle only; the docstring gives re-running every cycle as the thing to avoid | §V-H `sections/method.tex`; `On-Track-SysID/src/on_track_sys_id.py:420` |
| `delta_max = 0.34 rad` | `sweep_delta_max: 0.54` | §V-E `sections/method.tex`; `On-Track-SysID/params/pacejka_params.yaml` |
| `W = 20` samples = 0.4 s | 20 × `sample_dt` 0.035 = 0.7 s | §V-A3 `sections/method.tex` |
| `T_s = 0.035 s`, "matched to the server's 0.033 s step" | 0.035 matches neither 0.033 nor 1/50 = 0.020; the yaml comment claims it must match 1/rate at 50 Hz, which is also wrong | §IV-E `sections/platform.tex`, A8 `sections/appendix.tex`; `On-Track-SysID/params/nn_params.yaml:3` |

**Do**

1. Rewrite §V-H's warm-start paragraph to describe the implemented behaviour: the warm
   start runs on the first identification cycle only, and every later re-identification
   cold-starts from the static `pacejka_model` D. Keep the safety rationale — it is the
   reason for the first-cycle-only design — but attach it to the right behaviour: because
   the warm start is both the solver's start point and, through the frozen prior, the
   regularisation target, recomputing it each cycle would rewrite the tire model, and
   through it the MPC's reference speed profile, under the running controller.
2. Correct `delta_max` to the shipped `0.54` rad, and re-check the surrounding argument:
   the sentence about `delta_max` lying above the plant's 0.279 rad mechanical lock still
   holds, but the margin changes.
3. Correct the window duration to 0.7 s at `T_s = 0.035`, and re-justify the match to
   tire-relaxation timescales at that number, or change `sequence_length` and re-run.
4. State where `T_s = 0.035 s` comes from. If it is empirical, say so. Remove the claim
   that it is "matched to" the 0.033 s step, in both §IV-E and A8, and fix the misleading
   comment in `nn_params.yaml:3` while you are there.
5. In §IV-E, separate the two mismatches that are currently conflated: 0.035 against the
   0.033 physics step (6 %), and the 50 Hz acquisition timer against the 0.033 step (65 %).
   The zero-order-hold argument belongs to the second.

**Accept** Every configuration value quoted in §IV, §V and A8 matches
`On-Track-SysID/params/*.yaml` verbatim. Verify by grepping each quoted value.

### F2. "Structural identifiability" is the wrong term

**Evidence** §III-C `sections/problem.tex`, and every downstream use of the phrase.

**Do** The Magic Formula is structurally identifiable given data that reaches the peak;
what fails here is *practical* (a-posteriori) identifiability under a specific excitation.
Rename §III-C to "Practical identifiability under limited excitation". Keep Bellman and
Åström as the structural-identifiability contrast, not as the citation for the observed
phenomenon. Add Raue et al., "Structural and practical identifiability analysis of
partially observed dynamical models by exploiting the profile likelihood",
*Bioinformatics* 25(15):1923–1929, 2009, as the reference for the distinction.

**Accept** No sentence in the paper calls the observed degeneracy structural
non-identifiability.

### F3. Disclose the prior's provenance (paired with F7)

**Evidence** `On-Track-SysID/params/pacejka_params.yaml` header: `C_Pf_model` /
`C_Pr_model` were obtained by fitting CARLA's own per-wheel telemetry
(`/sim/feedback/tire_forces`) over a limit-excitation run reaching α = 0.37 rad and axle
μ = 1.05 — i.e. from the same channel §VI scores against, over a manoeuvre the pipeline is
not allowed to use. `graphs/comparison/comparison_summary.csv` shows the `ours` row's eight
coefficients are bit-identical to that prior.

**Do** Say this explicitly in §VI-A ("Setup"), not only in §V-F's passing reference to
"the measured tire of Table II". One short paragraph: where the prior came from, what
manoeuvre produced it, and that it is derived from the validation channel. Then soften the
abstract, contribution 6 and the conclusion so that they claim what §VI-D already concedes
— the corrected pipeline *preserves* a physically valid peak factor where the inherited
configuration destroys it — rather than claiming the peak factor was identified.

**Accept** A reader can tell, from §VI alone, that the reported μ RMSE of 0.041/0.048 is
the accuracy of a prior the fit did not move.

### F4. Numerical and notational nits

**Do**, each traced to its artefact:

- §I contribution 2: "1.01 → 1.87 drift" mixes the front prior with the rear final value.
  Write it per axle: 1.009 → 1.5402 front, 1.002 → 1.8721 rear.
- Table I: 2644.5 N / 9.81 = 269.6 kg, not 269.8. Fix the mass, or state the `g` used, and
  propagate — the value appears in Table I, §I, §V-F and Table VII.
- §I: "driven at 9.6–25.8 m/s" is the reference speed profile; the achieved identification
  data spans 9.2–26.5 m/s (§V-A2). Distinguish the two.
- §VII gate 3 says "per 100-stage horizon" while §VII's opening says `N = 20`. Reconcile.
- §V-H says the identifier runs at 20 Hz; §IV-E says the buffer is sampled by a 50 Hz
  timer. State which is the node rate and which the acquisition timer.
- Unify `delta_sweep` and `delta_max` notation between the abstract, Eq. (14) and §I-1.
- Cut the abstract from 265 to ≤ 250 words.
- Note that the baseline's understeer margin is 2 % (`l_f C_f` = 14.10 against
  `l_r C_r` = 14.38 kN·m/rad) — it nearly trips gate 2, which strengthens the gate argument.
- Eq. (14) uses the Ackermann curvature δ/L, so it is an *upper* bound on the demand the
  rollout makes; the true steady-state curvature is δ/(L + K_us v²/g). State it as a bound.
  In Table V, note that `mu_reach` values far above D (10.64 at 20 m/s) mean "far past
  saturation", not a demand the tire ever sees.
- Table V's caption pools "five noise realisations × two axles" into ten fits. Two axles
  from one rollout are not independent, so the CI half-widths are optimistic. Report
  per-axle, or compute the interval over the five realisations only.
- §VI-B: at 6 m/s, `mu_reach` = 0.96 < D = 1.05 yet the 1.05 tire is recovered to 1 %. Say
  the boundary is soft.
- Figure 8 (`pacejka_identified_vs_nominal`): shade or mark the 2.2° slip band the lap
  actually visits. It is the whole argument of §VI-E and currently lives only in prose.
- Re-export `paper/figures/fig2.png` at ≥ 600 dpi (it is 524 × 457 px, ≈175 dpi at the
  printed width) or drop the figure.
- Fix `LaTeX Font Warning: Font shape OT1/ptm/m/scit undefined` (a `\textsc` nested inside
  `\emph`).

### F5. References

**Do**

- `physxvehicle`: pin the exact PhysX SDK version CARLA 0.9.16 / UE 4.26 links.
  `PxVehicleComputeTireForceDefault` is the PhysX 4 vehicle API, but the entry carries
  `year = {2018}` and a PhysX 5 Omniverse URL. Eq. (12) is transcribed from a specific
  version; name it and cite the matching documentation.
- `ahn2013friction`: the title is "Robust Estimation of Road **Frictional** Coefficient",
  IEEE TCST 21(1):1–13, 2013. Add the DOI.
- `hewing2020cautious`: complete to IEEE TCST 28(6):2736–2743, 2020, with DOI.
- `forzaeth2024`, `ontrack2025`, `hewing2018cautious`: add volume/pages/DOI where they exist.
- Re-verify `visionsysid2026` (arXiv 2603.09399) and `simfidelity2026` (arXiv 2602.07984)
  against arXiv immediately before submission — §II-C's whole overlap disclosure hangs on
  the first, so a wrong ID there is maximally damaging.
- Replace the branch URL in the Data-and-Code Availability section and in Appendix B with a
  tagged release, and mint Zenodo DOIs for `carlaasubridge` and for this repository.
- Name the `On-Track-SysID` upstream licence explicitly instead of "inherits its licence".
- Add the prior work the review flags as missing, each where it belongs in §II:
  Khaleghian, Emami & Taheri, "A technical survey on tire-road friction estimation",
  *Friction* 5(2), 2017 (beside `acosta2017friction`); Cabrera, Ortiz, Simón et al., "An
  alternative method to determine the magic tyre model parameters using genetic
  algorithms", *VSD* 41(2), 2004 (supports the multi-start and DE choices in §V-F);
  Åström & Wittenmark, *Adaptive Control*, 2nd ed. (persistent excitation, beside Ljung);
  Sierra, Tseng, Jain & Peng, "Cornering stiffness estimation based on vehicle lateral
  dynamics", *VSD* 44(sup1), 2006 (the recursive comparator §II-B argues against).

---

## Phase 2 — Runs

Each item below is a measurement. None is a writing task. Run them, export the artefacts,
then write §VI from the artefacts.

### F6. Repetitions and confidence intervals

**Evidence** Every row of Table VI is a single run per arm. Limitation 1 concedes it, while
the abstract, §VI-J and the conclusion state "halves the lateral tracking error" as if
established.

**Do** ≥ 5 seeds per arm via `benchmark_runner/scenarios.yaml`. Report mean ± CI, or a
paired test, on every directional row of Table VI. Set `pacejka_solver.seed` in both
parameter sets first (see F11) so the arms are reproducible.

**Accept** No directional claim in the paper rests on a single observation, and Limitation 1
is rewritten to describe what is now measured.

### F7. Show what the data contributes to D

**Evidence** `ours` returns the prior to four decimal places on all six accepted
identifications, so the data's contribution to D is indistinguishable from zero — even
though Table V says D is recoverable to < 4 % once `mu_reach` > 1.7.

**Do**

1. Sweep the prior: run `Ours` from `C_P*_model` D of 0.6, 1.0, 1.5 and 2.0, everything
   else fixed, and report where each lands. If D tracks the prior, that is the result.
2. Run `Ours` once with `prior_weight: 0.0` to separate the regulariser's contribution from
   the data's.
3. Log and report the realised `mu_reach` per identification cycle for both arms — add it
   as a column to Table VI. `delta_sweep = 1.5 · P99(|delta_obs|)` with
   `P99(|delta|) ≈ 0.06` rad gives `delta_sweep ≈ 0.09` rad, so at the 15 m/s floor
   `mu_reach ≈ 1.35` — marginal by the paper's own ≥ 1.7 criterion, and Table VII's
   "≥ 1.2 after correction" understates the problem.

**Accept** §VI states, with evidence, how much of the corrected pipeline's D comes from the
prior and how much from the rollout, and Table VII's reachable-μ row reflects the realised
value rather than a configuration bound.

### F8. Real baselines

**Evidence** The only comparator is `Baseline-NN-MSE`, which §VI-A3 admits is the inherited
*settings* on the *corrected* rollout, not the published pipeline. The published method's
own headline metric (3.3× lower one-step RMSE than NLS) is never reproduced, and the
abstract's "converges on a tire with less than half the vehicle's peak grip" is supported
only by the synthetic sweep and an offline anecdote.

**Do** Add three arms:

1. Nonlinear least squares on the same 30 s buffer — the baseline's own comparator.
2. A dual or joint EKF arm (`wenzel2006dekf`). §II-B already argues a filter suffers the
   same excitation condition and expresses it as a covariance that does not shrink;
   demonstrating that is a strong result in its own right.
3. The genuinely inherited pipeline in CARLA — rollout speed clipped to 4 m/s — so the
   abstract's failure claim rests on a controlled run at full scale and not on the
   synthetic sweep alone.

**Accept** The paper compares against at least two established methods from the literature
plus the inherited pipeline as published.

### F9. Multi-step prediction at the horizon the MPC uses

**Evidence** §VI-I: skill against persistence is −2.0/−5.4 (`Ours`) and −1.4/−12.6
(baseline) — both identified models lose to "next state equals last state". The defence
(one-step on a smooth line rewards not moving) is correct but incomplete: the MPC runs
`N = 20` at 20 Hz, a 1 s horizon.

**Do** §V-I says a benchmark node already computes 1/5/10-step metrics with Euler/Heun/RK4.
Report the 10-step and horizon-length RMSE and R² against persistence at the same horizon,
for both arms, and put them in Table VI beside the one-step rows.

**Accept** The causal chain "better tire model → better MPC tracking" is supported by a
prediction metric at the horizon the controller actually uses.

### F10. The friction warm start on the plant

**Evidence** `method: brush_axle` is active for `Ours` and `estimate_mu_brush()` is called,
but D came out exactly at the prior, so the gate must have rejected the brush fit and
fallen back. `pacejka_params.yaml` records that the brush estimator returned μ = 1.71/1.58
against the 1.00–1.05 measured on this vehicle, and that because the warm start is a floor
that bias can only push D up. §VI-H says only that "the released estimate [is] below the
configured prior … the floor never binds", which is a different statement. Table VIII is a
brush model fitted to a brush plant — self-matched structure, four rows identical to three
decimals, no noise column, and its lowest utilisation (45 %) sits 5 points above the 40 %
gate.

**Do**

1. Log, per identification cycle of both CARLA runs, whether the brush fit was released and
   which of the three gate conditions failed when it was not. Report it in §VI-H.
2. Report the μ the brush estimator returns on this vehicle against the measured 1.00–1.05,
   and attribute the bias to the inputs of Eq. (19) it is linear in (mass, `I_z`). This is
   an empirical property of the estimator, not a development anecdote, and it belongs in
   the paper.
3. Add a mismatched-plant row to Table VIII: fit the brush model to the PhysX curve of
   Eq. (12), where the structure is not self-matched.
4. Sweep a biased `mu_hat` over the synthetic brush plant and report the gate's pass rate
   against the bias.

**Accept** The reader knows what contribution 3 did on the reported runs, and the only
evidence for it is no longer a self-matched synthetic fit.

### F11. Reproducibility runs

**Do**

- Set `pacejka_solver.seed` in both parameter sets and re-run, so the reported
  identifications are bit-reproducible. Update A8, which currently discloses the gap.
- Re-profile the S4D identification with per-phase timers so both columns of Table IX sum
  to their totals. The "after" stage rows currently reconstruct 21.1 s against a 20.9 s
  end-to-end total.
- Note in §V-F that `num_starts: 8` is ignored by differential evolution, so the
  multi-start claim does not apply to the shipped `Ours` arm.
- Read `I_z` from the CARLA actor's inertia tensor. The carried 51.1 kg·m² implies a radius
  of gyration of 0.435 m; `m · l_f · l_r` for this vehicle is ≈158 kg·m², and measured
  Formula Student cars land at 90–130. It enters Eq. (2) directly, scales the yaw residual
  the network trains on, enters `F_y,f` at the 10 % level through Eq. (19) — and therefore
  the brush μ of F10 — and sets the sub-stepping of gate 3. Either measure it and re-run,
  or report the sensitivity of every affected result to it. Update A1 either way.

### F12. Strengthen §VI-B

**Evidence** The sweep varies rollout speed at a fixed `delta_sweep = 0.4` rad, so the claim
"the boundary is `mu_reach`, not a speed" rests on varying D only. It also zeroes the
residual network, so it fits a Magic Formula to Magic-Formula data — the best possible case.

**Do** Extend `paper/figures/make_identifiability_sweep.py` to vary `delta_sweep` at fixed
`v` and show the same collapse at the same `mu_reach`. Add one sentence noting that with
the residual network zeroed the model structure is exactly matched, so the real pipeline's
degradation is at least this bad.

**Accept** §VI-B demonstrates that `mu_reach` — not speed — is the governing variable, along
both of its axes.

---

## Phase 3 — Structure and venue

### F13. Cut to ≤ 14 pages and 3–4 contributions

**Evidence** 26 pages against an IEEE T-IV regular-paper length of roughly 10–14; six
contributions in §I-B.

**Do**

- Merge contributions 2 and 3 (both are corrections implied by the bound). Demote
  contribution 4 (the architecture) and the S4D cost half of contribution 5 to supporting
  material. Target: the identifiability bound and the corrections it implies; the
  admissibility gates; the architecture and its ground truth.
- Move §V-A3–A4 and Table IX (the S4D residual and its cost) to supplementary material or
  to a companion paper. Nothing in the paper's central argument depends on them.
- Move Table VII to supplementary.
- Compress §II-C to about four sentences. The disclosure is right to be there; at its
  current length it outweighs the contribution it disclaims.
- Move the §IV-D interface detail to the appendix.
- Add a pseudocode block for the corrected identification loop. It replaces about two pages
  of prose and it is what a reader needs in order to reimplement.

### F14. Title and keywords

**Do** The title does not say the work is in simulation, and every result is. Use something
like "… at Full Scale: A Simulation Study" or "… in a Force-Instrumented Simulator". Draw
the keywords from IEEE T-IV's own taxonomy.

---

## Ordering

Phase 1 first — it is free and it removes the contradictions a reviewer would find in
minutes. Then F6, F7 and F9 together, since they share a benchmark sweep. Then F8 and F10,
which need new arms. F12 is independent of the simulator and can run at any time. Phase 3
last, once the content is final.

If a phase-2 run cannot be completed, say so explicitly in the paper and in
`REVIEW_TRACKING.md` rather than leaving the affected claim standing at its current
strength.
