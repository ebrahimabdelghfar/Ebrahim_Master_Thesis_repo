# Paper update note — peak-friction identification (2026-09-14)

Working note for the next paper pass. Not paper text. The paper should state the
method and its physical rationale; the implementation history stays in `.wolf/`.

---

## 1. The result that drives every edit below

`analyse_tires()` reconstructs the axle forces the Pacejka fit sees as

```
F_yf / F_zf = v_x·ω / (g·cos δ)        F_yr / F_zr = v_x·ω / g
```

so the objective of Eq. (fitobj) depends on the coefficients only through the
product `B·C·D`. The peak factor `D` is the free coordinate of that hyperbola:
the fit cannot separate it from `B`, and the `B ≥ 4.0` box edge decides where it
stops.

Measured on the five identification buffers of the live μ = 1.05 run, six
iterations each:

| quantity | across the five cycles | plant |
|---|---|---|
| `B·C·D·F_zf` (front cornering stiffness) | 18.9 – 21.1 kN/rad | **20.4 kN/rad** |
| `D` reaching that same product | 0.69 – 2.00 | 1.05 |

The pipeline identifies **cornering stiffness**, accurately and repeatably, from
a prior of 13.2 kN/rad. It does not identify `D`. Iteration 1's `D` equals the
peak μ the synthetic rollout itself realises (reach 1.50 → 1.497, 1.15 → 1.125,
0.68 → 0.858); since each iteration's `D` becomes the next rollout's nominal and
reach increases with `D`, the loop is positive feedback about an unstable fixed
point near `D ≈ 1.0` — it runs up to the 2.0 bound above it and down to 0.4
below it.

**Control worth quoting in the paper:** no configuration that lets `D` move beats
returning the prior. Mean |D − 1.05| over the five buffers — prior 0.041;
Tikhonov λ=2.0 0.046 (indistinguishable from the prior); λ=0.5 0.220; a
10 %/iteration cap on ΔD 0.271; a single iteration 0.282; the shipped six 0.585.
Freezing `D` alone does not help either — the degeneracy relocates to `C`, which
rails at 2.200.

## 2. What the method now is

`D` is no longer asked of the rollout fit. It is measured by the brush estimator
(§V-F), which fits the **recorded** trace and is therefore not subject to the
`B·C·D` degeneracy, and is then **held fixed** for the whole co-identification
loop while `B`, `C`, `E` carry the stiffness the rollout does determine.

Four consequences for the method text:

1. **The per-axle μ is applied two-sided, not as a floor.** The 2026-09-01 floor
   rule was protecting against a `D` that *moved* between cycles, not against a
   `D` that was low. Pinning gives that protection directly and without the
   one-way bias, which a low-μ road makes untenable.
2. **The measured `D` is pinned, not seeded.** A seed is overwritten within one
   cycle: a 0.785 seed ended the live run at 2.000.
3. **An axle with no measured μ holds the `D` it came in with.** An unpinned
   axle does not "fall back to fitting `D`" — it walks to a box edge.
4. **The warm start runs every cycle.** `nn_train` re-reads the static prior at
   the start of each cycle, so a first-cycle-only warm start does not hold `D`
   still; it leaves every later cycle starting from a `D` unrelated to the road.

Measured effect, same five buffers, mean |D − 1.05| / max / front-stiffness
spread:

| | mean | max | C_α spread |
|---|---|---|---|
| free `D` (what §VI currently reports) | 0.585 | 0.950 | 32.9 kN/rad |
| pin only the measured axles | 0.191 | 0.378 | 32.3 |
| **pin, and hold the rest** | **0.084** | **0.213** | **18.5** |

`D` is now flat across all six iterations on every cycle.

### Two supporting corrections

- **Front axle only.** Rear μ reads 0.25–0.45 high (1.495/1.383/1.257 against
  1.011/0.964/0.837) because the rear slip angle does not carry δ, so the rear
  curve has too little curvature to identify μ from. This is *not* a yaw-inertia
  error: in quasi-steady cornering `F_yf/F_yr → l_r/l_f`, the same ratio as
  `F_zf/F_zr`, so μ_f = μ_r whatever `I_z` is — swept 30–80 kg·m² and the axle
  gap only widens, 0.368 → 0.550. One road surface, so the identifiable axle
  speaks for both.
- **The excitation gate now tests the absolute `|F_y|/F_z`.** The published gate
  divides by the fitted μ, so an axle whose μ comes out too low reports a high
  utilisation and clears the gate on its own error. On the five buffers the
  absolute ratio separates the usable from the unusable exactly: {0.146, 0.314}
  rejected, {0.541 … 0.677} accepted.

## 3. Paper edits required

### Method — rewrite

| passage | what it says now | what it must say |
|---|---|---|
| `sections/method.tex:370-385` "Applied as a floor, unconditionally" | `D` can only ever be raised, never lowered; the floor is the property the interface needs | the per-axle μ is applied two-sided and then held fixed; what the interface needs is that `D` not *move*, which pinning supplies directly |
| `sections/method.tex:33-62` Algorithm 1, lines 3–5 | `if first cycle`; `D⁰ ← max(D⁰, μ̂)` (floor only) | every cycle; `D ← μ̂_f` for both axles when gated, else hold; `D` fixed for the loop |
| Algorithm 1, lines 14–15 | `argmin` over all four coefficients | `argmin` over `B, C, E` with `D` held |
| `sections/method.tex` §V-F gate list | utilisation gate only | add the absolute `\|F_y\|/F_z` condition and why the ratio form is self-referential |
| §V-F, rear axle | per-axle μ released per axle | front axle speaks for both; give the `l_r/l_f` argument, not an inertia argument |
| New short subsection in §IV or §V | — | the `B·C·D` identifiability statement of §1 above. This is the physical justification for everything else and the paper currently lacks it |

### Results — must be re-run, not edited

`sections/experiments.tex:444-467` (§VI "Peak friction") reports that `Ours`
returns `D_f = 1.009`, `D_r = 1.002` in **every** one of six identifications,
"six identical coefficient sets to four decimal places", μ RMSE 0.041 against
the baseline's 0.762, and frames this as the pipeline *preserving* a valid peak
factor.

Those six identical sets are the signature of a fit that did not execute — the
rollout diverged, `analyse_tires` admitted fewer than eight samples, and
`solve_pacejka` returned its input coefficients unchanged, six times. The
0.041 is the configured prior being republished, and the 16–19× advantage over
the baseline is not a result. **Discard and re-run.** Affected:

- `sections/experiments.tex:347` Table VI, "Front μ RMSE 0.041 / 0.762"
- `sections/experiments.tex:127-133`, `:444-467` §VI-? Peak friction, whole subsection
- `sections/experiments.tex:473` Fig. μ-timeseries and its caption
- `sections/abstract.tex:21` "preserves a peak factor within 0.05"
- `sections/intro.tex:118`, `sections/conclusion.tex:23-26` the same claim
- `graphs/comparison/` — regenerate with `compare_scenarios.py` after the sweep,
  never while `pgrep -f run_benchmark.py` matches

Unaffected: §VI-B / Table V (its own rollout, not `nn_train`) and every
`Baseline-NN-MSE` number — that arm runs `friction_warm_start.enable: false`, so
none of these changes touch it and it remains the control.

### Supplement

- `sections/supplement.tex:333` config table row: "always a floor" → two-sided
  and pinned for the per-axle fit; floor-only for the utilisation fallback
- `sections/supplement.tex:322` rollout speed row is unchanged
- SVI-A already records that those runs predate earlier corrections; extend it

### Tracking

`REVIEW_TRACKING.md:12` records F1.1 "warm start is first-cycle-only" as **done**
with the first-cycle gate as evidence. That gate is now deliberately removed;
re-open the row with the reasoning of §2.4 above.

## 4. What must happen before the results can be written

Re-run `benchmark_runner/run_benchmark.py` for both arms. Every `Ours` number in
§VI predates all of these changes.

**Run at more than one μ.** This is the single most important point in this note.
At a single plant μ the configured prior scores 0.041 purely because 1.009 sits
near 1.05, while being incapable of tracking anything — a one-μ table makes the
corrected pipeline look *worse* than doing nothing. The claim the method actually
supports is adaptivity, and it is only visible across μ. The brush estimator
returns 0.747 on the real plant-0.70 buffer, where the prior is wrong by 0.31.
The friction schedules already written in `benchmark_runner/scenarios.yaml`
(`step40_end_lap1`, `decay_2pct_s`) are the intended vehicle for this.

## 5. Open author decisions

1. **Headline claim.** "Preserves a valid peak factor" is no longer the right
   framing, since `D` now moves on evidence. Candidates: (a) the pipeline
   identifies cornering stiffness to ~1 % and tracks μ across surfaces; (b) keep
   the preservation framing and report stiffness as the identified quantity.
   (a) is stronger and is what the measurements support, but needs the multi-μ
   sweep to stand up.
2. **Whether to report `B·C·D·F_z` as the headline identified quantity.** It is
   accurate (18.9–21.1 against 20.4) and repeatable across cycles, and the paper
   currently reports nothing of the kind.
3. **`num_of_iterations`.** Stiffness needs ~5–8 iterations to reach 20.4 kN/rad;
   iteration 1 alone leaves it at 13.5–14.6. With `D` pinned the iterations no
   longer damage it, so 6 is now defensible on evidence rather than inherited.
