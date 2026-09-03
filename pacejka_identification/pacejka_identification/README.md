# `pacejka_identification` — Source Modules

This directory contains the core Python modules that implement the Pacejka Magic Formula coefficient identification pipeline.

---

## Module Overview

| Module | Purpose |
|--------|---------|
| `magic_formula.py` | Pure-math Pacejka equations (no ROS2 dependency) |
| `coefficient_identifier.py` | Optimisation engine with 8 methods (incl. Bayesian SVI) + built-in GA/JADE |
| `data_collector_node.py` | ROS2 node — subscribes to `/sim/feedback/tire_forces`, filters & buffers data |
| `identification_node.py` | ROS2 node — orchestrates collect → identify → publish → export |

```
                    ┌──────────────────────┐
                    │ /sim/feedback/       │
                    │      tire_forces     │
                    │ (sim_manager_msgs)   │
                    └─────────┬────────────┘
                              │
                    ┌─────────▼────────────┐
                    │  identification_node │
                    │                      │
                    │  Phase 1: Collect    │──▶ CSV export
                    │  Phase 2: Identify   │
                    │  Phase 3: Publish    │──▶ YAML export
                    └──┬───────────┬───────┘
                       │           │
          ┌────────────▼──┐   ┌────▼──────────────────┐
          │ magic_formula │   │ coefficient_identifier│
          │   .py         │   │   .py                 │
          │               │   │                       │
          │ pacejka_fy()  │   │ CoefficientIdentifier │
          │ pacejka_fx()  │◀──│ GeneticAlgorithm      │
          │ pacejka_mz()  │   │                       │
          └───────────────┘   └───────────────────────┘
```

---

## `magic_formula.py`

Implements the standard Pacejka "Magic Formula" tire model:

```
Y(x) = Fz · D · sin(C · arctan(B·x − E·(B·x − arctan(B·x))))
```

### Coefficient Meaning

| Symbol | Name | Physical Role |
|--------|------|---------------|
| **B** | Stiffness factor | Controls the slope at zero slip (cornering stiffness = B·C·D) |
| **C** | Shape factor | Controls the overall shape of the curve (number of peaks) |
| **D** | Peak factor | Normalised peak force/torque (peak = D·Fz) |
| **E** | Curvature factor | Controls the curvature near the peak (E ∈ [-2, 2]) |

### Functions

| Function | Inputs | Description |
|----------|--------|-------------|
| `pacejka_formula(params, slip, fz)` | `[B,C,D,E]`, slip variable, normal load | General formula (used internally) |
| `pacejka_fy(params, alpha, fz)` | `[B,C,D,E]`, slip angle α [rad], Fz [N] | Lateral force Fy |
| `pacejka_fx(params, kappa, fz)` | `[B,C,D,E]`, slip ratio κ [-], Fz [N] | Longitudinal force Fx |
| `pacejka_mz(params, alpha, fz)` | `[B,C,D,E]`, slip angle α [rad], Fz [N] | Self-aligning torque Mz |

All functions accept and return NumPy arrays (vectorised).

---

## `coefficient_identifier.py`

The optimisation engine that fits [B, C, D, E] to measured data.

### Classes

#### `CoefficientIdentifier`

Main class for coefficient identification.

**Constructor Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `method` | `'dual'` | Optimiser: `trust_region`, `differential_evolution`, `dual`, `genetic_algorithm`, `ga_trust_region`, `adaptive_de`, `adaptive_de_trust_region`, `bayesian_svi` |
| `identification_mode` | `'sequential'` | `sequential` (fix D to the data peak, fit B,C,E) or `simultaneous` (fit all 4) |
| `fixed_C` | `None` | MAP prior mean / sequential C seed, e.g. `{'fy': 1.3}` |
| `lower_bounds` | `[0.1, 0.1, 0.01, -2.0]` | Lower bounds for [B, C, D, E] |
| `upper_bounds` | `[50.0, 5.0, 5.0, 2.0]` | Upper bounds for [B, C, D, E] |
| `regularization` | `'none'` | Simultaneous-mode only: `'none'` or `'map'` (literature bounds + Gaussian prior on C) |
| `map_reg_params` | `{'lambda_C': 1.0, 'sigma_C': 0.3}` | MAP penalty weight / prior std-dev for C |
| `svi_params` | `{'num_steps': 2000, 'learning_rate': 0.01, 'seed': 42}` | Hyperparameters for `bayesian_svi` |
| `data_balancing_params` | `{'enabled': False, 'n_bins': 20, 'max_per_bin': 200}` | Optional slip-histogram rebalancing before fitting |

**Public Methods:**

| Method | Signature | Description |
|--------|-----------|-------------|
| `identify()` | `(slip, fz, y_meas, initial_guess, label)` → `(coeffs, metrics)` | General identification |
| `identify_fy()` | `(alpha, fz, fy, initial_guess)` → `(coeffs, metrics)` | Lateral force shorthand |
| `identify_fx()` | `(kappa, fz, fx, initial_guess)` → `(coeffs, metrics)` | Longitudinal force shorthand |
| `identify_mz()` | `(alpha, fz, mz, initial_guess)` → `(coeffs, metrics)` | Self-aligning torque shorthand |

**Return values:**
- `coeffs`: `[B, C, D, E]` list of identified coefficients
- `metrics`: dict with `R2`, `RMSE`, `MAE`, `MaxAE`, `n_samples`, `residual_norm`

#### `GeneticAlgorithm`

A custom real-coded Genetic Algorithm for global optimisation.

**GA Features:**

| Feature | Implementation |
|---------|---------------|
| Selection | Tournament selection (configurable size) |
| Crossover | BLX-α blend crossover (α = 0.5) |
| Mutation | Per-gene Gaussian mutation (adaptive σ) |
| Elitism | Top 5% preserved across generations |
| Population | 120 individuals, 400 generations |

### Identification Modes

#### Sequential Mode *(recommended for ground truth)*

Fixes D — the friction coefficient μ — to the (bounds-clipped) binned peak
estimate read directly off the data, seeds B from the cornering-stiffness
slope (BCD/(C·D)) near the origin, then fits B, C and E. Fixing D is what
breaks the dominant part of the Magic Formula's parameter degeneracy —
leaving D free to float (as a prior version of this code did) reopens a B-D
correlation that a "sequential" fit is specifically meant to close. C is left
free, so a residual B-C-E correlation remains.

C's seed comes from the configured initial guess, falling back to the
channel's literature value:

| Channel | C seed | Literature Range |
|---------|--------|------------------|
| Fy (lateral) | 1.30 | 1.1 – 1.8 |
| Fx (longitudinal) | 1.65 | 1.4 – 1.8 |
| Mz (self-aligning) | 2.40 | 2.0 – 3.0 |

#### Simultaneous Mode

Fits all 4 parameters at once. Excellent curve fit (R² > 0.999) but
B, C, E values may not be unique due to parameter correlation. Two optional,
opt-in anti-overfitting mechanisms are available (see `regularization` /
`method='bayesian_svi'` above), both grounded in Goblirsch et al., "Bayesian
Optimization-based Tire Parameter and Uncertainty Estimation for Real-World
Data", TUM, arXiv:2504.20863 (2025):

- **`regularization='map'`** — literature-consistent bounds (`LITERATURE_BOUNDS`)
  plus a Gaussian-prior penalty anchoring C toward its literature value
  (classical MAP estimate, no extra dependency).
- **`method='bayesian_svi'`** — full Stochastic Variational Inference via
  Pyro: a correlated multivariate-normal posterior over the free parameters,
  trained against the ELBO, returning both a point estimate and `*_std`
  uncertainty for each parameter.

Both approaches also benefit from optional slip-histogram rebalancing
(`data_balancing_params`), which prevents a dense near-zero-slip cluster from
dominating the fit at the expense of the sparser near-peak region.

### Optimisation Methods Comparison

| Method | Type | Speed | Accuracy | Best For |
|--------|------|-------|----------|----------|
| `trust_region` | Local | ★★★★★ | ★★★ | Clean data with good initial guess |
| `differential_evolution` | Global | ★★★ | ★★★★ | Unknown parameter ranges |
| `dual` | Hybrid (DE→TR) | ★★★ | ★★★★★ | **Ground-truth identification** |
| `genetic_algorithm` | Global | ★★ | ★★★★ | Exploring broad solution space |
| `ga_trust_region` | Hybrid (GA→TR) | ★★ | ★★★★★ | Alternative to dual |
| `adaptive_de` | Global (JADE) | ★★★ | ★★★★ | Self-tuning global search |
| `adaptive_de_trust_region` | Hybrid (JADE→TR) | ★★★ | ★★★★★ | Alternative to dual |
| `bayesian_svi` | Global (SVI) | ★★ | ★★★★ | Uncertainty-aware ground truth |

### Helper Functions

| Function | Description |
|----------|-------------|
| `_residuals(params, slip, fz, y_meas)` | Element-wise residual vector |
| `_cost(params, slip, fz, y_meas)` | Sum-of-squares cost (scalar) |
| `_residuals_BE(be, C, D, slip, fz, y_meas)` | Residuals with C and D fixed |
| `_residuals_reg(params, ..., C_lit, lambda_C, sigma_C)` | Residuals + Gaussian-prior penalty on C (MAP) |
| `_metrics(params, slip, fz, y_meas)` | Compute R², RMSE, MAE, MaxAE |
| `_estimate_D_from_peak(slip, fz, y_meas)` | Estimate D from 99th percentile |
| `_estimate_cornering_stiffness(slip, fz, y_meas)` | Estimate BCD slope near the origin (seeds B) |
| `_rebalance_slip_bins(slip, fz, y_meas, n_bins, max_per_bin)` | Subsample dense slip bins before fitting |

---

## `data_collector_node.py`

Standalone ROS2 node that subscribes to `/sim/feedback/tire_forces` and collects filtered data.

### Class: `DataCollectorNode(Node)`

**ROS2 Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tire_forces_topic` | `/sim/feedback/tire_forces` | Input topic (`sim_manager_msgs/TireForces`) |
| `duration_seconds` | `60` | Collection window (seconds of wall clock from the first message) |
| `odom_topic` | `/odom` | Speed source for the standstill gate (`''` disables it) |
| `min_speed` | `2.0` | Min speed to accept a sample (m/s); `0.0` disables the gate |
| `min_fz_threshold` | `50.0` | Min normal force filter (N) |
| `csv_export` | `True` | Auto-export dataset on completion |
| `csv_path` | `''` | Output path (auto-generated if empty) |

**Data Filtering:**

Each incoming sample is checked against:
1. Vehicle speed from `odom_topic` ≥ `min_speed` (per message, not per wheel)
2. `|Fz|` ≥ `min_fz_threshold`
3. Not a standstill sample — the publisher zeroes slip and both forces below
   0.5 m/s while `normal_load` stays live, so filter 2 does not catch them
4. No NaN/inf values in any field

**Stored Data (per wheel: FL, FR, RL, RR):**

`slip_angle`, `slip_ratio`, `fz`, `fy`, `fx`, `wheel_torque`, `tire_friction`, `wheel_speed`

**Data Access Methods:**

| Method | Description |
|--------|-------------|
| `get_numpy(wheel, column)` | Single wheel, single column → `ndarray` |
| `get_axle_numpy(axle, column)` | `'front'`/`'rear'` pooled → `ndarray` |
| `get_all_numpy(column)` | All 4 wheels pooled → `ndarray` |
| `is_complete` | Property — True when collection is done |
| `export_csv()` | Write dataset to CSV file |

---

## `identification_node.py`

Main ROS2 node that orchestrates the full coefficient identification pipeline.

### Class: `IdentificationNode(Node)`

Operates in three sequential phases:

```
Phase 1: COLLECTING    → subscribe to /sim/feedback/tire_forces, buffer per-wheel data
Phase 2: IDENTIFYING   → run CoefficientIdentifier for each formula × group
Phase 3: DONE          → publish results (latched), export YAML + CSV
```

**ROS2 Parameters:**

All parameters from `DataCollectorNode` plus:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `method` | `'dual'` | Optimisation method |
| `identification_mode` | `'sequential'` | `sequential` or `simultaneous` |
| `formulas` | `[lateral_fy]` | Which channels to identify (`self_aligning_mz` is unavailable — the message has no Mz) |
| `axle_grouping` | `'per_axle'` | `per_wheel`, `per_axle`, or `combined` |
| `initial_guess_fy` | `[10, 1.5, 1.0, 0.5]` | Starting point for Fy |
| `initial_guess_fx` | `[10, 1.65, 1.0, 0.5]` | Starting point for Fx |
| `lower_bounds` | `[0.1, 0.1, 0.01, -2.0]` | Parameter lower bounds |
| `upper_bounds` | `[50.0, 5.0, 5.0, 2.0]` | Parameter upper bounds |
| `yaml_export` | `True` | Export coefficients to YAML |
| `yaml_path` | `''` | YAML output path (auto if empty) |

**Axle Grouping:**

| Mode | Groups | Description |
|------|--------|-------------|
| `per_wheel` | FL, FR, RL, RR | Separate coefficients per wheel |
| `per_axle` | front, rear | Pool left+right per axle |
| `combined` | all | Pool all 4 wheels |

**Published Topics:**

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `/pacejka_id/coefficients` | `String` | Latched | YAML-formatted coefficients |
| `/pacejka_id/fit_metrics` | `String` | Latched | Summary with R², RMSE |
| `/pacejka_id/status` | `String` | Default | Phase + progress |

**Subscribed Topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `/sim/feedback/tire_forces` | `sim_manager_msgs/TireForces` | Ground-truth per-wheel telemetry from CARLA |
| `/odom` | `nav_msgs/Odometry` | Speed for the standstill gate (only if `min_speed > 0`) |

**Exported Files:**

| File | Content |
|------|---------|
| `~/pacejka_id_data_<timestamp>.csv` | Raw dataset (per-wheel columns) |
| `~/pacejka_id_coefficients_<timestamp>.yaml` | Identified [B,C,D,E] + metrics |
