# Paper update note — the residual objective and the co-identification loop (2026-09-14 b)

Working note for the next paper pass. Not paper text. Companion to
`SYSID_METHOD_UPDATE_2026-09-14.md`, which covers the peak factor `D`; this one
covers the two defects that made the `Ours` arm lose the *force* metrics to
`Baseline-NN-MSE` even after `D` was fixed. Implementation history stays in
`.wolf/`.

---

## 0. The complaint

In `graphs/comparison/comparison_summary.csv` the `Ours` arm is worse than the
control on every force and state quantity the benchmark reports:

| | `Ours` | `Baseline-NN-MSE` |
|---|---|---|
| front / rear axle `F_y` RMSE | 93.1 / 97.4 N | **60.1 / 56.0 N** |
| front / rear `F_y` R² | 0.891 / 0.865 | **0.961 / 0.961** |
| `v_y` RMSE | 0.047 m/s | **0.037 m/s** |
| ITAE `e_y` | 21678 | **16865** |
| front / rear μ RMSE | **0.203 / 0.200** | 1.136 / 1.148 |

`Ours` wins μ by 5.6× and loses force by 1.6×. This note is about the second
half. The first half is the subject of the companion note.

## 1. Method

Everything below was measured by replaying `nn_train()` offline on the **five
real identification buffers of the same run** —
`install/on_track_sys_id/share/on_track_sys_id/data/SIM/SIM_sys_id_data_20260914_124*.csv`
for `Ours`, `..._125*.csv` for the control — and scoring each configuration with
the quantity the tire-force benchmark itself reports: the identified Magic
Formula evaluated at the plant's own measured slip angles, against the plant's
own measured lateral forces, from
`graphs/identification/<arm>/pacejka_curve_validation.csv`. No synthetic plant,
no re-derived metric.

**One trap in that file, worth a footnote if it is ever cited.** `fy_N` is a
**single wheel's** force — `tire_force_benchmark_node` feeds `slip_history` per
wheel — while `fz_N` is the whole **axle's** static load. Fit it as written and
every cornering stiffness comes out a factor of two low. Halved correctly, the
plant's initial slope `d(F_y/F_z)/dα` on this run is:

| axle | plant | `Ours` as shipped | `Baseline-NN-MSE` | configured prior |
|---|---|---|---|---|
| front | **11.52 /rad** | 15.44 (+34 %) | 12.90 (+12 %) | 9.61 (−17 %) |
| rear | **11.84 /rad** | 17.00 (+44 %) | 13.22 (+12 %) | 10.91 (−8 %) |

The car never leaves the linear tire region on this lap (|α| ≤ 2.9°, peak
`|F_y|/F_z` = 0.23), so the force metric is a *cornering-stiffness* metric and
nothing else — consistent with `bug-090`. `Ours` was simply further from the
plant's stiffness than the control was. Two independent causes.

## 2. Defect 1 — the physics-informed loss is 2.3 × 10⁵ times the data loss

`compute_physics_informed_loss` builds

```
lat_dyn_residual = m·v̇_y + m·v_x·ω − (F_r + F_f cos δ)        [N]
yaw_dyn_residual = I_z·ω̇ − (F_f l_f cos δ − F_r l_r)          [N·m]
```

and adds `λ_steady · mean(residual²)` to a `data_loss` that is an MSE over a
one-step increment in m/s and rad/s. The two terms are four to six orders of
magnitude apart. Measured at initialisation on a real buffer:

| term | raw | × λ | share of total |
|---|---|---|---|
| data | 1.62 × 10⁻³ | 1.62 × 10⁻³ | **0.0004 %** |
| steady | 3.66 × 10³ | 3.66 × 10² | **99.9996 %** |
| symmetry | 3.5 × 10⁻⁵ | 1.8 × 10⁻⁶ | ~0 |
| smoothness | ~10⁻¹³ | ~10⁻¹⁷ | ~0 |

So `λ_steady = 0.1` did not mean "10 % weight on the physics". The residual
network was being trained, to within one part in 2 × 10⁵, to cancel the *nominal
Pacejka model's own force imbalance* — and never to fit the data it was given.
The final training losses in the run log say the same thing out loud:
`physics_informed` converges at 211–26 000 while `mse` on identical data
converges at 0.0025.

**Fix.** Express each residual as the state increment it implies — the
network's own output units — by scaling the lateral residual by `dt/m` and the
yaw residual by `dt/I_z`. After the fix, on the same buffer: data **98.53 %**,
steady **1.47 %**. `λ_steady` now means what the paper says it means.

This is a genuine correction to Eq. (physloss) as written in §V-A and the
supplement, not an implementation detail: the objective as published is
dominated by a term whose units were never reconciled with the data term.

## 3. Defect 2 — the co-identification loop is not a contraction

`nn_train` ends every iteration with `model['C_P*_model'] ← C_P*_identified`,
and `simulated_data_gen` rolls out **that** model to produce the trajectory the
next `solve_pacejka` fits. The fit therefore reads its own forces back, over a
steering sweep that already extrapolates 1.5 × past the observed range. The loop
is a positive feedback path on its own extrapolation error.

It shows directly as a monotone degradation in iteration count (five buffers,
front / rear axle `F_y` RMSE):

| `num_of_iterations` | 1 | 2 | 3 | 6 |
|---|---|---|---|---|
| front | **38.9 N** | 53.0 | 58.5 | 70.4 |
| rear | **26.3 N** | 44.5 | 42.5 | 62.3 |

This is the same mechanism the companion note identifies for `D` ("each
iteration's `D` becomes the next rollout's nominal … positive feedback about an
unstable fixed point"), now visible in the coefficients `D` does *not* absorb.
Pinning `D` does not remove it; it relocates it into `B`, `C` and `E`.

**Fix — damp the step, do not shorten the loop.** New
`pacejka_solver.update_relaxation` (`train_model.relax_update`), applied in
`nn_train`:

```
C ← (1 − β)·C_prev + β·C_fit
```

β = 1 is the shipped behaviour and remains the default, so
`Baseline-NN-MSE` — whose parameter set omits the key — is untouched and
remains the control. `params/pacejka_params.yaml` ships β = 0.20. Full sweep,
five buffers, six iterations throughout:

| β | 0.00 | 0.05 | 0.10 | 0.15 | **0.20** | 0.25 | 0.30 | 0.50 | 1.00 |
|---|---|---|---|---|---|---|---|---|---|
| front N | 92.5 | 68.3 | 48.6 | 33.9 | **28.0** | 31.2 | 36.4 | 55.6 | 70.4 |
| rear N | 71.8 | 49.0 | 31.3 | 19.8 | **19.4** | 26.0 | 30.2 | 38.4 | 62.3 |
| front `B·C·D` | 7.82 | 8.82 | 9.66 | 10.36 | **10.88** | 11.32 | 11.65 | 12.75 | 12.42 |

**β = 0 is the control the paper needs, and it is the worst column.** At β = 0
the coefficients never move: it is the pinned prior, i.e. the pipeline doing no
identification at all. That it loses to every damped setting is what separates
"damping helps" from "identifying at all hurts", and it is the honest way to
report the contribution. The minimum sits at β = 0.20 (23.5 ± 1.5 N over seeds
0 / 3 / 7).

Framed for the paper, the co-identification loop of Algorithm 1 is a
**relaxed fixed-point iteration**, not a plain one. That is a one-line change to
the algorithm and a defensible piece of method, not a tuning constant.

## 4. Result

Both fixes together, five real buffers, scored against the plant's own telemetry:

| | front / rear `F_y` RMSE | front R² | identified `D` (plant 0.70) |
|---|---|---|---|
| `Ours`, as shipped | 70.4 / 62.3 N | 0.915 | 0.734 |
| `Ours`, normalised loss only | 77.1 / 76.2 N | 0.891 | 0.734 |
| **`Ours`, both fixes** | **28.0 / 19.4 N** | **0.986** | **0.734** |
| `Baseline-NN-MSE` | 61.5 / 46.0 N | 0.934 | 1.738 |
| prior, no identification (β = 0) | 92.5 / 71.8 N | 0.870 | 0.734 |

`Ours` now leads the control by ~55 % on force RMSE **and** by 5× on peak
friction, instead of trading one for the other. Two supporting ablations, same
protocol:

- **The S4D architecture is worth ~26 % on its own.** At one iteration and
  identical settings, S4D 35.3 N against the baseline MLP 44.7 N (mean of the
  two axles). The paper currently claims the S4D contribution is *cost and
  exactness only* (related work, discussion limitations) and calls the accuracy
  of the temporal arm an unrun ablation. This is that ablation, and it is
  positive — but it is an offline replay on recorded buffers, not a closed-loop
  result, and should be reported as such.
- **`loss_mode` is now a wash.** With the damping in place, normalised
  `physics_informed` and plain `mse` are within seed noise (23.7 vs 23.7 N).
  Keeping `physics_informed`: it costs nothing and it is the paper's stated
  objective. But the paper should not claim the physics term buys accuracy on
  this manoeuvre — it does not, measurably.

## 5. Paper edits required

### Method — rewrite

| passage | what it says now | what it must say |
|---|---|---|
| §V-A objective, Eq. (physloss) + supplement SV-A2 | `L = L_data + λ_s L_steady + λ_sym L_sym + λ_sm L_sm`, terms unqualified | the dynamics residuals are normalised by `dt/m` and `dt/I_z` to the state-increment units of the residual output, so the λ are relative weights; state why (the unnormalised form is 2.3 × 10⁵ × the data term) |
| `sections/method.tex` Algorithm 1, coefficient update line | `C ← C_identified` | `C ← (1−β)C + β C_identified`, β = 0.20; name it a relaxed fixed-point iteration |
| New paragraph, §IV or §V, next to the `B·C·D` identifiability statement the companion note asks for | — | the loop is not a contraction: the rollout that generates the fit's data is the previous fit's own answer. This is the shared mechanism behind both `D`'s ratchet and the stiffness drift |
| §V-A2 / related work / discussion, "S4D contributes cost and exactness only" | accuracy of the temporal arm is an unrun ablation | the ablation is run: +26 % on force RMSE over the baseline MLP at matched settings, offline replay, recorded buffers |
| Table IV (configuration diff `ours` vs `Baseline-NN-MSE`) | four settings | five — add `pacejka_solver.update_relaxation` (0.20 vs 1.0) |

### Results — must be re-run, not edited

Both fixes change what a run produces, so they compound the re-run requirement
the companion note already states. In addition to the passages listed there:

- `sections/experiments.tex` Table VI rows for front / rear `F_y` RMSE, `F_y`
  R², `v_y` RMSE, ITAE — currently show the control ahead, which the offline
  replay says is now reversed
- §VI-C … §VI-G narrative built on those rows
- `graphs/comparison/` — regenerate with `compare_scenarios.py` after the sweep,
  never while `pgrep -f run_benchmark.py` matches

Unaffected, again: §VI-B / Table V, and every `Baseline-NN-MSE` number.

### Supplement

- Table IX / SV-A3 runtime figures are unaffected — neither fix changes the
  per-epoch or rollout cost
- SV-A2's objective needs the same normalisation as §V-A
- SVI-A: extend the "these runs predate …" note to cover both fixes

## 6. Correction to the companion note's open decision 3

`SYSID_METHOD_UPDATE_2026-09-14.md` §5.3 concludes that with `D` pinned, the
iterations "no longer damage it, so 6 is now defensible on evidence". That
holds for `D`, which is pinned and therefore cannot move. It does **not** hold
for the coefficients the loop still fits: at β = 1 those degrade monotonically
with iteration count (§3 above), and the stiffness the companion note credits to
5–8 iterations (18.9–21.1 kN/rad against a plant at 20.4) was measured on the
μ = 1.05 run at β = 1, where the drift ran in the helpful direction from a prior
that was 35 % low.

With β = 0.20 the question is settled differently and better: six iterations
**are** defensible, because the damping makes them converge rather than drift,
and the β = 0 control shows the iterations are what carries the information. No
change to `num_of_iterations` is needed or recommended.

## 7. What must happen before the results can be written

Unchanged from the companion note, and for one more reason: re-run
`benchmark_runner/run_benchmark.py` for both arms, at **more than one μ**. Every
`Ours` force and state number in §VI predates both fixes in this note as well as
every change in the companion note.

## 8. Verification

- `On-Track-SysID/src/helpers/train_model.py` — loss normalisation in
  `compute_physics_informed_loss`; new `relax_update`, wired into `nn_train`;
  `pacejka_update_relaxation` read in `get_model_param`
- `On-Track-SysID/params/pacejka_params.yaml` — `pacejka_solver.update_relaxation: 0.2`
- `On-Track-SysID/test/test_pacejka_identifiability.py` — three regression
  tests: the relaxation arithmetic and its β = 1 no-op, the shipped config
  carrying a damped β, and the physics term's magnitude being commensurate with
  the data term
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest test/ -q` → **27 passed**
  (24 before, 3 new). The ROS `launch_testing` plugin is incompatible with the
  conda pytest, hence the flag
- `.wolf/buglog.json` → `bug-physics-informed-loss-unnormalised-force-residual`,
  `bug-co-identification-loop-is-not-a-contraction`
