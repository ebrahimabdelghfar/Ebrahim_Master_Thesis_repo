# `pacejka_identification` — Source Modules

This directory contains the core Python modules that implement the Pacejka Magic Formula coefficient identification pipeline.

---

## Module Overview

| Module | Purpose |
|--------|---------|
| `magic_formula.py` | Pure-math Pacejka equations (no ROS2 dependency) |
| `coefficient_identifier.py` | Optimisation engine with 5 methods + built-in GA |
| `data_collector_node.py` | ROS2 node — subscribes to `/tire_forces`, filters & buffers data |
| `identification_node.py` | ROS2 node — orchestrates collect → identify → publish → export |

```
                    ┌──────────────────────┐
                    │  /tire_forces topic  │
                    │  (hellocm_msgs)      │
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
| `method` | `'dual'` | Optimiser: `trust_region`, `differential_evolution`, `dual`, `genetic_algorithm`, `ga_trust_region` |
| `identification_mode` | `'sequential'` | `sequential` (fix C, fit B,D,E) or `simultaneous` (fit all 4) |
| `fixed_C` | `None` | Override default C values, e.g. `{'fy': 1.3}` |
| `lower_bounds` | `[0.1, 0.1, 0.01, -2.0]` | Lower bounds for [B, C, D, E] |
| `upper_bounds` | `[50.0, 5.0, 5.0, 2.0]` | Upper bounds for [B, C, D, E] |

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

Fixes C to physics-based literature values, then fits B, D, E only.
This breaks the known B-C-E parameter degeneracy.

| Channel | Fixed C | Literature Range |
|---------|---------|------------------|
| Fy (lateral) | 1.30 | 1.1 – 1.8 |
| Fx (longitudinal) | 1.65 | 1.4 – 1.8 |
| Mz (self-aligning) | 2.40 | 2.0 – 3.0 |

#### Simultaneous Mode

Fits all 4 parameters at once. Excellent curve fit (R² > 0.999) but
B, C, E values may not be unique due to parameter correlation.

### Optimisation Methods Comparison

| Method | Type | Speed | Accuracy | Best For |
|--------|------|-------|----------|----------|
| `trust_region` | Local | ★★★★★ | ★★★ | Clean data with good initial guess |
| `differential_evolution` | Global | ★★★ | ★★★★ | Unknown parameter ranges |
| `dual` | Hybrid (DE→TR) | ★★★ | ★★★★★ | **Ground-truth identification** |
| `genetic_algorithm` | Global | ★★ | ★★★★ | Exploring broad solution space |
| `ga_trust_region` | Hybrid (GA→TR) | ★★ | ★★★★★ | Alternative to dual |

### Helper Functions

| Function | Description |
|----------|-------------|
| `_residuals(params, slip, fz, y_meas)` | Element-wise residual vector |
| `_cost(params, slip, fz, y_meas)` | Sum-of-squares cost (scalar) |
| `_residuals_BDE(bde, C, slip, fz, y_meas)` | Residuals with C fixed |
| `_metrics(params, slip, fz, y_meas)` | Compute R², RMSE, MAE, MaxAE |
| `_estimate_D_from_peak(slip, fz, y_meas)` | Estimate D from 99th percentile |

---

## `data_collector_node.py`

Standalone ROS2 node that subscribes to `/tire_forces` and collects filtered data.

### Class: `DataCollectorNode(Node)`

**ROS2 Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `tire_forces_topic` | `/tire_forces` | Input topic |
| `duration_seconds` | `60` | Collection window (seconds) |
| `min_velocity` | `0.5` | Min belt velocity filter (m/s) |
| `require_on_road` | `True` | Discard samples when tire is airborne |
| `min_fz_threshold` | `50.0` | Min normal force filter (N) |
| `csv_export` | `True` | Auto-export dataset on completion |
| `csv_path` | `''` | Output path (auto-generated if empty) |

**Data Filtering:**

Each incoming sample is checked against:
1. `on_road` flag (if `require_on_road` is True)
2. `|Fz|` ≥ `min_fz_threshold`
3. `belt_velocity` ≥ `min_velocity`
4. No NaN values in key fields

**Stored Data (per wheel: FL, FR, RL, RR):**

`slip_angle`, `long_slip`, `fz`, `fy`, `fx`, `mz`, `inclination_angle`, `mu_road`

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
Phase 1: COLLECTING    → subscribe to /tire_forces, buffer per-wheel data
Phase 2: IDENTIFYING   → run CoefficientIdentifier for each formula × group
Phase 3: DONE          → publish results (latched), export YAML + CSV
```

**ROS2 Parameters:**

All parameters from `DataCollectorNode` plus:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `method` | `'dual'` | Optimisation method |
| `identification_mode` | `'sequential'` | `sequential` or `simultaneous` |
| `formulas` | `[lateral_fy, longitudinal_fx, self_aligning_mz]` | Which channels to identify |
| `axle_grouping` | `'per_axle'` | `per_wheel`, `per_axle`, or `combined` |
| `initial_guess_fy` | `[10, 1.5, 1.0, 0.5]` | Starting point for Fy |
| `initial_guess_fx` | `[10, 1.65, 1.0, 0.5]` | Starting point for Fx |
| `initial_guess_mz` | `[10, 1.5, 0.1, 0.5]` | Starting point for Mz |
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
| `/tire_forces` | `TireForcesArray` | Ground truth from CarMaker |

**Exported Files:**

| File | Content |
|------|---------|
| `~/pacejka_id_data_<timestamp>.csv` | Raw dataset (per-wheel columns) |
| `~/pacejka_id_coefficients_<timestamp>.yaml` | Identified [B,C,D,E] + metrics |
