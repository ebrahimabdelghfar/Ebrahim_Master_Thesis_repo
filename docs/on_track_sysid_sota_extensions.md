# On-Track-SysID: S4D temporal residual + non-vision friction warm-start

Two additive, switchable extensions to `On-Track-SysID` beyond its base paper (Dikici et al. 2024,
arXiv:2411.17508) and its existing physics-informed/ensemble/multi-arch extensions (see
`On-Track-SysID/README.md`): a temporal (state-space) residual model as an alternative to the existing
memoryless MLP architectures, and a sensor-based cold-start prior for the Pacejka `D` (peak-friction)
coefficient. Both are off by default in a way that changes nothing for existing configs — `s4` is one
more `nn_architecture` enum value, `friction_warm_start` only fires once, before the first identification
cycle.

Both are motivated by a 2026 follow-on to this package's own base paper — "Vision-Augmented On-Track
System Identification for Autonomous Racing via Attention-Based Priors and Iterative Neural Correction"
(arXiv:2603.09399) — which upgrades the residual model to an S4 state-space model and adds a camera-based
friction prior for cold-start warm-starting. This work reimplements the temporal-residual idea faithfully
(as S4D, see below) and reimplements the warm-start idea **without any camera/vision component**, since
this package has no vision pipeline.

## 1. S4D temporal-residual architecture (`nn_architecture: s4`)

### Why S4D, not S4

The cited paper's residual model is "S4" — but the actual S4 (Gu et al., *Efficiently Modeling Long
Sequences with Structured State Spaces*, arXiv:2111.00396) evaluates its recurrence via an FFT/Cauchy-
kernel convolution, machinery built for sequences of thousands-to-millions of steps. This pipeline's
training sequences are ~1500 samples (a `data_collection_duration` of 30s at 50Hz) windowed down to 20-
step slices — three orders of magnitude short of where that machinery earns its keep. **S4D** (Gu, Goel &
Ré, *On the Parameterization and Initialization of Diagonal State Space Models*, arXiv:2206.11893) keeps
the same core idea — a complex-diagonal, HiPPO-initialized state-transition matrix — but is evaluated with
a plain causal recurrent scan, which is what `helpers/s4_residual.py` implements. Calling this "S4" would
overstate the match to the cited paper's exact algorithm.

### Discretization and initialization

Each of `H` independent single-input-single-output channels carries an `N`-dimensional complex state.
Zero-order-hold discretization of the continuous-time system gives an exact closed form for diagonal `A`:

$$
\bar{A} = e^{A \cdot \Delta t}, \qquad \bar{B} = \frac{\bar{A} - 1}{A}
$$

Since `Re(A) < 0` is enforced by construction (`A_re = -exp(log_A_real)`, always negative under gradient
descent), `|\bar{A}| = e^{Re(A)\Delta t} < 1` for any `\Delta t > 0` — the layer is unconditionally stable,
with no extra output clipping needed. State index `n = 0 \ldots N-1` is initialized per the closed-form
diagonal approximation to HiPPO-LegS ("S4D-Lin"):

$$
\mathrm{Re}(A_n) = -0.5 \quad (\text{uniform exponential memory decay}), \qquad \mathrm{Im}(A_n) = \pi n \quad (\text{linearly increasing oscillation frequency})
$$

`B` is fixed at 1 (not learned — halves parameter count for negligible expressiveness loss at this scale,
since `C` is fully learnable). `C` (complex, per channel/state) and `D` (real skip/feedthrough, per
channel) are learned. `\Delta t` is a per-channel learnable discretization step, log-uniform initialized
in `[dt_min, dt_max]` (`nn_params.yaml`'s `s4.dt_min`/`s4.dt_max`) — a free SSM hyperparameter, **not** the
same quantity as `nn_params.yaml`'s `sample_dt` (the physical 1/rate integration step used by
`generate_predictions.py` and the physics-informed loss).

All complex arithmetic is hand-expanded into real `(*_re, *_im)` tensor pairs (no `torch.complex64`), to
avoid any dtype/autograd friction from partial complex-tensor support in this codebase's torch version.

### Two call contracts (`helpers/s4_residual.py`)

- `S4DResidual.forward(x)`: `x` shaped `(batch, W, d_in)` → `(batch, 2)`, the residual predicted at the
  window's last timestep. Identical contract to every other architecture's `nn_model(X_train)` call —
  `train_residual_nn()` and `compute_physics_informed_loss()` needed no changes to their model-calling
  code.
- `S4DResidual.init_state(batch)` / `.step(x_t, state)`: single-timestep stateful API, used only by
  `simulated_data_gen()` to carry hidden state across its 500-step open-loop rollout.

Per the user's explicit scope decision, this model is **training-only** — `simulated_data_gen()`'s
stateful stepping and `train_residual_nn()`'s windowed training are both entirely inside `nn_train()`'s
offline co-identification loop. `on_track_sys_id.py`'s online runtime (`publish_estimates()`, published
topics) is unchanged; the trained S4D model is discarded at the end of each `nn_train()` call exactly like
every other architecture already is.

### Windowing

Sliding windows (`s4.sequence_length`, default 20 samples = 0.4s at 50Hz), stride 1, many-to-one, built by
`helpers/generate_inputs_errors.py::generate_sequence_windows()`. This is a deliberate choice over a
full-sequence (`batch_size=1`) scheme: the residual physics this model corrects (tire relaxation,
transients) has a short memory horizon, and a full-sequence scheme would discard the existing
`DataLoader`/minibatch machinery for one noisy, slow gradient update per epoch — incompatible with the
pipeline's "converge in under a minute" design point.

`process_data()` (in `train_model.py`) concatenates two physically-continuous segments — the filtered
trace and its sign-mirrored copy (`negate_data()`) — with exactly one corrupted sample at the boundary
between them (a one-step-ahead prediction target that jumps from the original trajectory into its mirror
image). `generate_sequence_windows()` windows each segment **independently** and drops that boundary
sample entirely, rather than letting a window straddle it.

### Config (`On-Track-SysID/params/nn_params.yaml`)

```yaml
nn_architecture: "s4"   # options: baseline | wide | physics_inputs | ensemble | s4

s4:
  state_dim: 4
  num_channels: 4
  num_layers: 1             # keep at 1 - ~30s of data, don't overfit a deep stack
  sequence_length: 20       # W: 20 @ 50Hz sample_dt = 0.4s
  dt_min: 0.001
  dt_max: 0.1
  use_physics_inputs: false # 6-dim input (adds slip angles) before the S4D projection
```

`nn_architecture: ensemble` and `nn_architecture: s4` are mutually exclusive (single enum value) — not
composable in this pipeline.

### Verification (ad-hoc smoke script, synthetic data, not committed)

Ran the full `nn_train()`-equivalent pipeline (`build_model` → `train_residual_nn` → `simulated_data_gen`
→ `solve_pacejka`) on a synthetic ~1500-sample trajectory (nominal-Pacejka rollout + small Gaussian
measurement noise) across three configs (`use_physics_inputs` off/on, `loss_mode` mse/physics_informed):

- All three ran to completion with no exceptions. Windowed batch shapes were `(2950, 20, 4)` /
  `(2950, 20, 6)` as expected (`~1500`-sample collection window, `W=20`, two mirrored segments each
  contributing `~1475` windows). Parameter counts: 102 (`use_physics_inputs: false`), 110 (`true`) —
  comparable to `baseline` (58), `physics_inputs` (74), `wide` (114), `ensemble` (174).
- Isolated gradient check: one `loss.backward()` through a windowed batch, all parameter gradients
  finite (no NaN from the hand-rolled complex division in `_discretize()`).
- Isolated stateful-stepping stress test: 600 consecutive `.step()` calls with varying random input —
  output and all state tensors stayed finite and bounded (`< 1e3`) throughout, consistent with the
  ZOH-stability argument above.
- Windowing unit check: tagged synthetic segments confirmed no window straddles the segment boundary,
  and output shapes matched the expected `2*(segment_len - W)` count exactly.
- `solve_pacejka()`'s output on the resulting simulated sweep was finite and within its own `[B,C,D,E]`
  bounds in every run; its `alpha_f.size < 8` degenerate-data fallback was never triggered.

## 2. Non-vision friction warm-start (`friction_warm_start` in `pacejka_params.yaml`)

### The estimator

`helpers/friction_warmstart.py` estimates a single scalar instantaneous friction utilization:

$$
\hat\mu = \mathrm{median}\left(\frac{\sqrt{a_x^2 + a_y^2}}{g}\right) \quad \text{over a low-slip subset of samples}
$$

Since the Pacejka formula used throughout this codebase is `F_y = F_z \cdot D \cdot \sin(\ldots)`, `D` is
already the dimensionless peak-friction coefficient — `\hat\mu` maps directly onto `D_f`/`D_r`'s initial
guess with no `F_z` rescaling. Both axles get the same value (whole-vehicle quantity; body-frame
acceleration alone can't disaggregate front/rear friction). The low-slip mask (tighter than
`solve_pacejka.py::analyse_tires()`'s own `|alpha_f|<=0.5`/`|alpha_r|<=0.2` curve-coverage filters,
defaults `0.15`/`0.08` rad) is needed because the kinematic accel approximation below is only trustworthy
in the near-linear slip region. Median (not mean) over the masked subset for robustness to the
finite-difference operation's outlier-prone noise.

### Two accel sources

- `finite_diff` (default): `a_x \approx \dot v_x - v_y \omega`, `a_y \approx \dot v_y + v_x \omega`,
  central-differenced over the already-collected `[v_x,v_y,\omega,\delta]` buffer. No new topic or
  sensor.
- `imu`: reads `sensor_msgs/Imu.linear_acceleration.x/y` directly (lower noise, no differentiation) via a
  buffer kept in exact lockstep with the main data buffer (appended at the same `collect_data()` ticks,
  zero-order-hold on the IMU's own publish rate).

**Why `finite_diff` is the default, not `imu`:** checked whether this repo's platform has usable IMU data
before choosing a design. `f1tenth_simulator/params.yaml` declares an `imu_topic: "/imu"` and
`f1tenth_simulator/node/simulator.cpp`'s `pub_imu()` does publish `sensor_msgs::msg::Imu` on it — but that
function (lines 762-771) is an unimplemented stub (`// TODO: make imu message`): the message is
default-constructed and published as-is, so `linear_acceleration` is always zero. Since SIM is the only
environment this work could actually run/validate against, a design that *required* IMU would be silently
broken in the one environment available here. Real NUC hardware may well have a working IMU; the `imu`
accel source is implemented and documented for that case, with a fallback to `finite_diff` (logged as a
warning) if `friction_warm_start.accel_source: imu` is set but zero IMU messages ever arrive by the time
the estimate is needed. `f1tenth_simulator`'s IMU stub itself was not fixed — flagged here as a separate,
pre-existing gap, out of scope for this work.

### Where it plugs in

Runs once — the node's first-ever identification cycle only (`on_track_sys_id.py`'s
`_is_first_identification` flag, checked and unconditionally cleared in `run_identification_cycle()`
right after the training attempt). `maybe_compute_warm_start_mu()` reads (and copies) `self.data` **before**
`run_nn_train()` — `train_model.py::filter_data()` mutates its `training_data` argument's columns in
place, and `self.data` is passed to `nn_train()` by reference, so reading after training would silently
see Butterworth-filtered data instead of raw odometry. The resulting `mu_hat` is passed through
`nn_train(..., warm_start_mu=mu_hat)`, which overrides `model['C_Pf_model'][2]`/`['C_Pr_model'][2]` (only
the `D` index — `B`/`C`/`E` are untouched) before the co-identification loop starts, clipped to
`solve_pacejka.py`'s own `PACEJKA_BOUNDS` `D` range.

Every subsequent `reidentification_interval_s` periodic retrain is unaffected — verified against the
actual call chain (not assumed): `get_model_param()` rebuilds `C_Pf_model`/`C_Pr_model` from the static
`pacejka_params.yaml` on *every* call including periodic retrains, so today even periodic retrains cold-
start from the static default already. This feature only ever helps the bootstrap cycle, as scoped; making
periodic retrains self-warm-start from their own previous fit would be a natural, low-risk follow-on, left
out of scope here.

### Config (`On-Track-SysID/params/pacejka_params.yaml`)

```yaml
friction_warm_start:
  enable: true
  method: friction_ratio
  accel_source: finite_diff    # finite_diff (default) | imu (real hardware only)
  imu_topic: "/imu"
  imu_wait_timeout_s: 2.0
  low_slip_alpha_f_max: 0.15
  low_slip_alpha_r_max: 0.08
  vx_min: 1.5
  omega_max: 5.0
  min_samples: 20
```

### Verification (ad-hoc smoke script, synthetic data, not committed)

**Accuracy.** Synthesized a noise-free steady-state turn (constant `\omega`, `v_y=0`, `\delta` chosen for
near-zero front slip) with a known `mu_true=0.3` (peak `|\alpha_r|=0.0562` rad, safely under the default
`0.08` threshold). `estimate_mu_from_buffer()` returned `mu_hat=0.3000` from 1498 low-slip samples — 0.00%
relative error (an oscillating-`\omega(t)` version of this same test measured a very different, much
lower `\hat\mu`, because the *median* of a time-varying `|\mu(t)|` is not its peak — the estimator was
correct and the first draft of the test's synthetic data was not; a constant-magnitude ground truth is the
right test for a median-based point estimate).

**Convergence speed.** Built a synthetic Pacejka sweep from a known true model with `D=0.6` (deliberately
far from the static yaml default `D≈0.388`), then ran `solve_pacejka()` twice on the identical synthetic
data: once starting from the static default `D`, once starting from the warm-started `D=0.6`. Total
`scipy.optimize.least_squares` function evaluations across both axles: **142 (static-default cold start)
vs. 42 (warm-started)** — a 70% reduction — with comparable final cost (`2.242 \times 10^{-1}` vs.
`2.279 \times 10^{-1}`, both effectively converged). This is the vision-free analog of arXiv:2603.09399's
"faster convergence" claim, measured directly on this repo's own solver rather than asserted by citation.

## Citations

- Dikici, Ghignone, Hu, Baumann, Xie, Carron, Magno, Corno. *Learning-Based On-Track System Identification
  for Scaled Autonomous Racing in Under a Minute.* arXiv:2411.17508, 2024. (Base paper this package
  implements.)
- *Vision-Augmented On-Track System Identification for Autonomous Racing via Attention-Based Priors and
  Iterative Neural Correction.* arXiv:2603.09399, 2026. (Source of the S4 temporal-residual idea and the
  vision-based warm-start idea this work reimplements non-vision.)
- Gu, Goel, Ré. *On the Parameterization and Initialization of Diagonal State Space Models.* arXiv:2206.11893,
  2022. (S4D — the actual architecture implemented in `s4_residual.py`.)
- Gu, Goel, Ré. *Efficiently Modeling Long Sequences with Structured State Spaces.* arXiv:2111.00396, 2021.
  (S4 — cited for contrast/scoping; see "Why S4D, not S4" above.)
- Gu, Dao, Ermon, Rudra, Ré. *HiPPO: Recurrent Memory with Optimal Polynomial Projections.* arXiv:2008.07669,
  2020. (Source of the HiPPO-LegS initialization used.)
- Oeltjen, Sobolewski, Faghfoorian, Domokos, Vidal, Yerramsetty, Ruchkin. *Online Slip Detection and Friction
  Coefficient Estimation for Autonomous Racing.* arXiv:2509.15423, 2026. (Non-vision precedent adapted for
  the friction warm-start estimator.)
- Pacejka, Bakker. *The Magic Formula Tyre Model.* Vehicle System Dynamics, 1992. (`D` coefficient's
  physical meaning as peak-friction ratio — already cited in this package's README.)
