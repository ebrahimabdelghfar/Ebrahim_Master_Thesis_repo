# pacejka_identification

Offline, direct identification of Pacejka Magic Formula tire-model coefficients
`[B, C, D, E]` from CarMaker-simulated ground-truth tire-force data
(`hellocm_msgs/TireForcesArray`). The identified coefficients are meant to
serve as **ground truth** for benchmarking the indirect, online NN+Pacejka
identification approach in `On-Track-SysID` (`On-Track-SysID/src/helpers/solve_pacejka.py`
fits the identical `[B,C,D,E]` formula from inverse-dynamics force estimates,
so the two packages' outputs are directly comparable).

Three ROS2 nodes/entry points:

| Entry point | Role |
|---|---|
| `identification_node` | Main node: collect → identify → publish → export (all-in-one) |
| `data_collector_node` | Standalone: subscribe + filter + buffer + CSV export only, no fitting |
| `test/test_identification.py` | Offline synthetic-data validation of the fitting engine (no ROS) |

## Notation

| Symbol | Description | Unit |
|--------|-------------|------|
| **Magic Formula** |||
| `x` | Slip variable — slip angle `alpha` (Fy, Mz) or slip ratio `kappa` (Fx) | rad / — |
| `Fz` | Normal (vertical) tire load | N |
| `B` | Stiffness factor — controls slope at zero slip (cornering stiffness = `B*C*D`) | — |
| `C` | Shape factor — controls overall curve shape | — |
| `D` | Peak factor — normalised peak force/torque (peak = `D*Fz`) | — |
| `E` | Curvature factor — controls curvature near the peak | — |
| **Per-tire raw fields** (`hellocm_msgs/TireForcesArray`) |||
| `slip_angle` | Lateral slip angle `alpha` | rad |
| `long_slip` | Longitudinal slip ratio `kappa` | — |
| `fz`, `fy`, `fx`, `mz` | Normal load, lateral force, longitudinal force, self-aligning torque | N, N, N, N·m |
| `inclination_angle` | Camber angle (collected, not currently used in fitting) | rad |
| `mu_road` | Road friction coefficient (collected, not currently used in fitting) | — |
| `on_road`, `belt_velocity` | Contact flag and wheel/tire speed, used only as sample filters | bool, m/s |
| **Sequential identification (recommended)** |||
| `C_literature` | Fixed literature value of `C` for the channel (`DEFAULT_C`: fy=1.30, fx=1.65, mz=2.40) | — |
| `D` (fixed) | 99th-percentile peak estimate of `\|y/Fz\|`, binned to average out noise before reading the peak | — |
| `B0` (seed) | Cornering-stiffness slope estimate `BCD/(C*D)` near `slip≈0`, used only as the optimizer's starting point | — |
| **Simultaneous MAP regularization** (`regularization: "map"`) |||
| `lambda_C` | Gaussian-prior penalty weight anchoring `C` toward `C_literature` | — |
| `sigma_C` | Prior std-dev for `C` (smaller = more tightly anchored) | — |
| **Bayesian SVI** (`method: "bayesian_svi"`) |||
| `num_steps` | SVI (ELBO) gradient-descent steps | — |
| `learning_rate` | Adam learning rate for the SVI guide | — |
| `*_std` | Posterior std-dev per parameter, reported alongside the point estimate | (units of that parameter) |
| **Data balancing** (`data_balancing.enabled`) |||
| `n_bins` | Number of equal-width bins across the observed slip range | — |
| `max_per_bin` | Samples kept per bin (over-represented bins are randomly subsampled) | — |

## 1. Sequential identification (recommended for ground truth)

Implements the classical Magic-Formula degeneracy-breaking procedure (Bakker/Pacejka-style):

1. **Fix `C`** to the literature value for the channel (`DEFAULT_C[fy|fx|mz]`).
2. **Fix `D`** — not merely seed it — to a peak estimate read directly off the
   data: bin the slip range, average `|y/Fz|` within each bin (to cancel
   per-sample noise), then take the max of the bin means. A raw per-sample
   99th-percentile estimate was tried first and rejected: any residual noise
   in a fixed `D` gets absorbed almost entirely by `E` during the fit
   (empirically ~15-20x amplification for this formula's shape near the
   peak), so `D`'s estimator has to be noise-robust, not just "reasonable".
3. **Seed `B`** from the cornering-stiffness slope `BCD/(C*D)` estimated by
   linear regression of `y/Fz` vs. slip within a small window around
   `slip=0` — only an initial guess, not fixed.
4. **Fit only `[B, E]`** (`C`, `D` held fixed) with the chosen optimizer.

Fixing *both* `C` and `D` (not just `C`) is what actually breaks the Magic
Formula's parameter non-uniqueness — leaving `D` free to float, as an earlier
version of this code did, reopens a `B`-`D` correlation that a "sequential"
fit exists specifically to close.

## 2. Simultaneous identification

Fits all of `[B, C, D, E]` at once. Best raw curve fit (R² > 0.999), but
individual `B`/`C`/`E` values may not be unique — different combinations can
reproduce nearly the same curve. Two independent, opt-in anti-overfitting
mechanisms are available, both grounded in Goblirsch, Ruhland, Betz,
Lienkamp, *"Bayesian Optimization-based Tire Parameter and Uncertainty
Estimation for Real-World Data"*, TUM, arXiv:2504.20863 (IEEE, 2025):

- **`regularization: "map"`** — switches to literature-consistent bounds
  (`LITERATURE_BOUNDS`, taken from the paper's Table I for Fy/Fx; Mz's bound
  is scaled down from that same table since its normalised peak is roughly
  an order of magnitude smaller and the paper doesn't cover Mz directly) and
  adds a Gaussian-prior penalty anchoring `C` toward its literature value —
  a classical MAP estimate, no extra dependency.
- **`method: "bayesian_svi"`** — full Stochastic Variational Inference via
  Pyro: an `AutoMultivariateNormal` guide over the free parameters, trained
  against the ELBO, returning a point estimate (posterior mean) plus a
  `*_std` uncertainty for each parameter. Usable in both sequential mode
  (uncertainty on `B`,`E` only, since `C`,`D` are fixed constants) and
  simultaneous mode (uncertainty on all four, and their correlations are
  what the paper's method actually captures). Requires the optional
  `pyro-ppl` dependency; slower than the other methods (default 2000 SVI
  steps).
- **`data_balancing.enabled: true`** (independent of the above, usable with
  any method/mode) — rebalances pooled slip samples into equal-width bins
  before fitting, so a dense near-zero-slip cluster (typical of real driving
  data) doesn't dominate the fit at the expense of the sparser near-peak
  region — directly implements the paper's "prevent overfitting of the
  linear region" technique.

## 3. Optimizer back-ends

All 8 selectable via `identification.method`:

| Method | Type | Notes |
|---|---|---|
| `trust_region` | Local (`scipy.optimize.least_squares`, TRF) | Fast, needs a good initial guess |
| `differential_evolution` | Global (`scipy.optimize.differential_evolution`) | Robust, no initial guess needed |
| `dual` | DE → TR hybrid | **Recommended** — best accuracy/speed trade-off |
| `genetic_algorithm` | Custom real-coded GA (tournament selection, BLX-α crossover, Gaussian mutation, elitism) | Global |
| `ga_trust_region` | GA → TR hybrid | Alternative to `dual` |
| `adaptive_de` | JADE (Zhang & Sanderson, 2009) — self-adaptive F/CR, current-to-pbest/1 mutation, external archive | Global, self-tuning |
| `adaptive_de_trust_region` | JADE → TR hybrid | Alternative to `dual` |
| `bayesian_svi` | Pyro SVI (see §2) | Only method that reports parameter uncertainty |

## 4. ROS2 interfaces

### Required topic

| | Topic (parameter) | Default | Type |
|---|---|---|---|
| Sub | `tire_forces_topic` | **`/tire_forces`** | `hellocm_msgs/msg/TireForcesArray` |

Both nodes (`identification_node`, `data_collector_node`) and the launch file
default to **`/tire_forces`** — this must match whatever republishes CarMaker's
tire-force feed as `hellocm_msgs/TireForcesArray` in the simulation (the same
name the sibling `tire_force_benchmark` and `estimation_benchmark` packages
default to). Override with `tire_forces_topic:=<name>` on the launch line if
your CarMaker bridge publishes it elsewhere.

The message is expected to expose `front_left`, `front_right`, `rear_left`,
`rear_right`, each with fields `slip_angle`, `long_slip`, `fz`, `fy`, `fx`,
`mz`, `inclination_angle`, `mu_road`, `on_road`, `belt_velocity` (see
Notation above).

### Published (identification_node only)

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/pacejka_id/coefficients` | `std_msgs/String` | Latched (`TRANSIENT_LOCAL`, depth 1, `RELIABLE`) | YAML-formatted `[B,C,D,E]` + metrics per group/formula |
| `/pacejka_id/fit_metrics` | `std_msgs/String` | Latched | Human-readable summary (R², RMSE, n) |
| `/pacejka_id/status` | `std_msgs/String` | Default (depth 10) | Current phase (`WAITING`/`COLLECTING`/`IDENTIFYING`/`DONE`) + progress |

`data_collector_node` publishes nothing — subscribe-and-export only.

## 5. Parameters

All declared as ROS2 parameters, settable via `config/identification_config.yaml`
or launch arguments; full defaults live in that file.

### Data collection

| Parameter | Default | Description |
|---|---|---|
| `tire_forces_topic` | `/tire_forces` | Input topic (see §4) |
| `data_collection.duration_seconds` | `60` | Collection window (seconds) |
| `data_collection.min_velocity` | `0.5` | Min `belt_velocity` to accept a sample (m/s) |
| `data_collection.require_on_road` | `true` | Discard samples where the tire is airborne |
| `data_collection.min_fz_threshold` | `50.0` | Min `\|Fz\|` to accept a sample (N) |

### Identification

| Parameter | Default | Description |
|---|---|---|
| `identification.method` | `dual` | One of the 8 optimizer back-ends (§3) |
| `identification.identification_mode` | `sequential` | `sequential` (fix C,D; fit B,E) or `simultaneous` (fit all 4) |
| `identification.regularization` | `none` | Simultaneous-mode only: `none` or `map` (§2) |
| `identification.formulas` | `[lateral_fy, longitudinal_fx, self_aligning_mz]` | Which channels to identify |
| `identification.axle_grouping` | `per_wheel` | `per_wheel` (FL,FR,RL,RR separately) / `per_axle` (front, rear pooled) / `combined` (all four pooled) |
| `identification.initial_guess_fy/_fx/_mz` | `[10.0,1.5,1.0,0.5]` / `[10.0,1.65,1.0,0.5]` / `[10.0,1.5,0.1,0.5]` | Starting `[B,C,D,E]` (trust_region, and seed for `dual`) |
| `identification.lower_bounds` / `upper_bounds` | `[0.1,0.1,0.01,-2.0]` / `[50.0,5.0,5.0,2.0]` | Bounds for `[B,C,D,E]` (simultaneous, `regularization="none"`) or effectively `[B,_,D,E]` (sequential — only indices 0,2,3 are used) |

### Optimizer hyperparameters

| Block | Parameter | Default | Description |
|---|---|---|---|
| `trust_region` | `max_nfev` | `10000` | Max function evaluations (TRF) |
| `differential_evolution` | `maxiter` | `1000` | Max DE generations |
| | `tol` | `1e-12` | Convergence tolerance |
| | `seed` | `42` | RNG seed (`-1` → non-deterministic) |
| | `polish` | `true` | L-BFGS-B local refinement after DE |
| `genetic_algorithm` | `pop_size` | `120` | Population size |
| | `n_generations` | `400` | Generations |
| | `crossover_rate` | `0.85` | BLX-α crossover probability |
| | `mutation_rate` | `0.15` | Per-gene Gaussian mutation probability |
| | `mutation_scale` | `0.10` | Mutation std-dev, fraction of parameter range |
| | `elite_frac` | `0.05` | Fraction preserved via elitism |
| | `tournament_size` | `3` | Tournament selection pool size |
| | `seed` | `42` | RNG seed |
| `adaptive_de` (JADE) | `pop_size` | `100` | Population size (NP) |
| | `n_generations` | `400` | Max generations |
| | `p` | `0.1` | Top-p fraction for pbest selection |
| | `c` | `0.1` | Adaptation rate for μF/μCR |
| | `archive_ratio` | `1.0` | External archive size, fraction of `pop_size` |
| | `seed` | `42` | RNG seed |
| `map_regularization` | `lambda_C` | `1.0` | Penalty weight anchoring `C` (simultaneous + `regularization="map"`) |
| | `sigma_C` | `0.3` | Prior std-dev for `C` |
| `bayesian_svi` | `num_steps` | `2000` | SVI/ELBO gradient steps |
| | `learning_rate` | `0.01` | Adam learning rate |
| | `seed` | `42` | RNG seed |
| `data_balancing` | `enabled` | `false` | Opt-in slip-bin rebalancing before fitting |
| | `n_bins` | `20` | Equal-width bins across the observed slip range |
| | `max_per_bin` | `200` | Max samples kept per bin |

### Output

| Parameter | Default | Description |
|---|---|---|
| `output.csv_export` | `true` | Export raw per-wheel dataset to CSV |
| `output.csv_path` | `""` (auto) | `~/pacejka_id_data_<timestamp>.csv` if empty |
| `output.yaml_export` | `true` | Export identified coefficients to YAML |
| `output.yaml_path` | `""` (auto) | `~/pacejka_id_coefficients_<timestamp>.yaml` if empty |

## 6. How to run

### Build

```bash
source /opt/ros/humble/setup.bash
cd /home/ebrahim/Ebrahim_Master_Thesis_repo
colcon build --packages-select pacejka_identification
source install/setup.bash
```

`pyro-ppl` (needed only for `method: "bayesian_svi"`) is a Python dependency,
not built by colcon — install once with `pip install pyro-ppl` (it pulls in
`torch`). The rest of the package only needs `numpy`/`scipy`/`pyyaml`.

### Quick start (defaults: 60s collection, `dual` method, `per_axle` grouping)

```bash
ros2 run pacejka_identification identification_node
```

### With the launch file (recommended — loads `config/identification_config.yaml`)

```bash
ros2 launch pacejka_identification pacejka_identification.launch.py \
    tire_forces_topic:=/tire_forces \
    duration_seconds:=90 \
    method:=dual \
    axle_grouping:=per_axle
```

Options not exposed as launch arguments (`regularization`, `map_regularization`,
`bayesian_svi`, `data_balancing`, per-channel initial guesses/bounds) are set
directly in `config/identification_config.yaml`.

### Standalone data collection only (no fitting)

```bash
ros2 run pacejka_identification data_collector_node --ros-args -p tire_forces_topic:=/tire_forces
```

### Inspect at runtime

```bash
ros2 topic echo /pacejka_id/status         # phase + progress
ros2 topic echo /pacejka_id/coefficients   # identified [B,C,D,E] (latched — arrives even for late subscribers)
ros2 topic echo /pacejka_id/fit_metrics    # R², RMSE summary (latched)
```

### Run the offline test suite (no ROS required)

```bash
cd pacejka_identification
python3 test/test_identification.py          # direct run — always works
python3 -m pytest test/test_identification.py -v   # if your pytest/plugin versions are compatible
```

Validates all 8 methods × both modes (+ `regularization="map"` and
`bayesian_svi` cases) against synthetic Pacejka data with known `[B,C,D,E]`,
checking parameter recovery (sequential) or curve-fit quality (simultaneous).

### Output files

After identification completes, exported to `~/` by default:

1. `pacejka_id_data_<timestamp>.csv` — raw per-wheel dataset
2. `pacejka_id_coefficients_<timestamp>.yaml` — identified `[B,C,D,E]` (+ `*_std` for `bayesian_svi`) with fit metrics

## References

- Pacejka, H.B., *"Tire and Vehicle Dynamics"*, 3rd Ed., Ch. 4 — Magic Formula and the classical sequential fitting procedure.
- Zhang, J. and Sanderson, A.C., *"JADE: Adaptive Differential Evolution with Optional External Archive"*, IEEE Trans. Evol. Comput., 2009 — `adaptive_de`/`adaptive_de_trust_region`.
- Goblirsch, Ruhland, Betz, Lienkamp, *"Bayesian Optimization-based Tire Parameter and Uncertainty Estimation for Real-World Data"*, TUM, arXiv:2504.20863 (2025) — `regularization="map"`, `bayesian_svi`, `data_balancing`.
