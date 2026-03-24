# Pacejka Magic Formula Coefficient Identification

## Overview

This ROS2 package performs **direct identification** of Pacejka Magic Formula tire model coefficients using ground-truth tire force data from the CarMaker simulation (published on `/tire_forces`).

The identified coefficients serve as **ground truth** for benchmarking other identification methods (e.g., the indirect NN-based approach in `On-Track-SysID`).

### Magic Formula

The standard Pacejka "Magic Formula" for a single force/torque channel:

```
Y(x) = Fz · D · sin(C · arctan(B·x − E·(B·x − arctan(B·x))))
```

Where:
- **x** = slip variable (slip angle α for Fy/Mz, slip ratio κ for Fx)
- **Fz** = vertical (normal) load
- **B** = stiffness factor
- **C** = shape factor
- **D** = peak factor
- **E** = curvature factor

## Features

| Feature | Description |
|---------|-------------|
| 3 force channels | Fy (lateral), Fx (longitudinal), Mz (self-aligning torque) |
| 3 identification methods | Trust-Region, Differential Evolution, Dual (DE→TR) |
| 3 grouping modes | per_wheel, per_axle, combined |
| Automatic CSV export | Raw dataset with per-wheel columns |
| Automatic YAML export | Identified [B,C,D,E] with fit metrics (R², RMSE) |
| Latched ROS2 topics | Coefficients available to late subscribers |

## Installation

```bash
cd ~/Ebrahim_Master_Thesis_repo
colcon build --packages-select pacejka_identification
source install/setup.bash
```

## Usage

### Quick Start

```bash
# Run with defaults (60s collection, dual method, per_axle grouping)
ros2 run pacejka_identification identification_node
```

### With Launch File

```bash
ros2 launch pacejka_identification pacejka_identification.launch.py \
    duration_seconds:=90 \
    method:=dual \
    axle_grouping:=per_axle
```

### Monitor Progress

```bash
# Watch identification status
ros2 topic echo /pacejka_id/status

# Get identified coefficients (latched)
ros2 topic echo /pacejka_id/coefficients

# Get fit quality metrics
ros2 topic echo /pacejka_id/fit_metrics
```

## Configuration

All parameters can be set via launch arguments or `config/identification_config.yaml`:

### General Settings

| Parameter | Default | Description |
|-----------|---------|-------------|
| `duration_seconds` | 60 | Data collection duration |
| `min_velocity` | 0.5 | Min belt velocity filter (m/s) |
| `min_fz_threshold` | 50.0 | Min normal force filter (N) |
| `method` | `dual` | See **Identification Methods** below |
| `axle_grouping` | `per_axle` | `per_wheel`, `per_axle`, `combined` |
| `formulas` | all 3 | `lateral_fy`, `longitudinal_fx`, `self_aligning_mz` |

### Algorithm Hyperparameters

Each optimizer has specific hyperparameters exposed in `config/identification_config.yaml`. The hybrid methods (`dual`, `ga_trust_region`, `adaptive_de_trust_region`) combine the parameters of both their respective global and local stages.

**Trust Region (`trust_region`)**
- `max_nfev` (10000): Maximum function evaluations.

**Differential Evolution (`differential_evolution`)**
- `maxiter` (1000): Maximum iterations.
- `tol` (1e-12): Convergence tolerance.
- `polish` (true): Local refinement via L-BFGS-B at the end.
- `seed` (42): Random seed (-1 for random).

**Genetic Algorithm (`genetic_algorithm`)**
- `pop_size` (120): Population size.
- `n_generations` (400): Maximum generations.
- `crossover_rate` (0.85): BLX-α crossover probability.
- `mutation_rate` (0.15): Gaussian mutation probability per gene.
- `mutation_scale` (0.10): Standard deviation of mutation (fraction of range).
- `elite_frac` (0.05): Fraction of population preserved (elitism).
- `tournament_size` (3): Selection pool size.
- `seed` (42): Random seed (-1 for random).

**Adaptive DE / JADE (`adaptive_de`)**
- `pop_size` (100): Population size.
- `n_generations` (400): Maximum generations.
- `p` (0.1): Top-p fraction for the *current-to-pbest/1* mutation.
- `c` (0.1): Adaptation rate for the F and CR distributions.
- `archive_ratio` (1.0): Size of external archive relative to population.
- `seed` (42): Random seed (-1 for random).

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/pacejka_id/coefficients` | `String` | YAML-formatted identified coefficients (latched) |
| `/pacejka_id/fit_metrics` | `String` | Fit quality summary (latched) |
| `/pacejka_id/status` | `String` | Current phase and progress |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/tire_forces` | `TireForcesArray` | Ground truth tire data from CarMaker |

## Output Files

After identification completes, two files are exported to `~/`:

1. **CSV** (`pacejka_id_data_<timestamp>.csv`) — raw dataset
2. **YAML** (`pacejka_id_coefficients_<timestamp>.yaml`) — identified coefficients with metrics

### Example YAML Output

```yaml
identification:
  method: dual
  formulas: [lateral_fy, longitudinal_fx, self_aligning_mz]
  axle_grouping: per_axle
coefficients:
  front:
    Fy:
      B: 7.1736
      C: 1.5628
      D: 0.3882
      E: 0.5278
      params: [7.1736, 1.5628, 0.3882, 0.5278]
      R2: 0.9987
      RMSE: 12.34
  rear:
    Fy:
      B: 8.2885
      C: 2.1091
      D: 0.3751
      E: 0.4036
      params: [8.2885, 2.1091, 0.3751, 0.4036]
      R2: 0.9992
      RMSE: 9.87
```

## Identification Methods

The `method` parameter allows you to select the backend optimization solver. The sequential mode breaks B-C-E coefficient parameter degeneracy, and these solvers find the optimal curve fit:

### 1. Trust-Region-Reflective (`trust_region`)
Local optimizer (TRF). Extremely fast but requires a good initial guess. Fails if the starting parameters are too far from the true curve. Best suited for clean, predictable data.

### 2. Differential Evolution (`differential_evolution`)
Classical global optimizer. Slower but robust against local minima. Reliable for identifying Pacejka curves when you have no good initial guess.

### 3. Dual (`dual`) — **Recommended**
Hybrid approach: runs Differential Evolution first for a robust global search, then passes the result to Trust-Region for extremely precise local refinement. Provides the highest overall accuracy.

### 4. Genetic Algorithm (`genetic_algorithm`)
Custom real-coded GA with tournament selection, BLX-α crossover, and Gaussian mutation. Operates strictly globally.

### 5. GA + Trust-Region (`ga_trust_region`)
Hybrid approach: runs the Genetic Algorithm first, then refines the output using Trust-Region. 

### 6. Adaptive DE / JADE (`adaptive_de`)
Advanced self-adaptive Differential Evolution (Zhang & Sanderson, 2009). Dynamically self-adapts its mutation (F) and crossover (CR) rates during optimization using Lehmer and arithmetic means respectively. Uses `current-to-pbest/1` mutation and an external archive of inferior solutions to maintain genetic diversity. Highly effective out-of-the-box global optimizer.

### 7. JADE + Trust-Region (`adaptive_de_trust_region`)
Hybrid approach: runs the Adaptive DE (JADE) first for robust, self-tuning global search, then refines the exact coefficients with Trust-Region.

## Architecture

```
identification_node
    │
    ├── Phase 1: Data Collection
    │   └── Subscribe to /tire_forces → buffer per-wheel arrays
    │
    ├── Phase 2: Identification
    │   └── CoefficientIdentifier.identify_fy/fx/mz()
    │       └── scipy.optimize.differential_evolution → least_squares
    │
    └── Phase 3: Publication & Export
        ├── Publish to /pacejka_id/coefficients (latched)
        ├── Export YAML with [B,C,D,E] + R², RMSE
        └── Export CSV raw dataset
```

## Package Structure

```
pacejka_identification/
├── config/
│   └── identification_config.yaml
├── launch/
│   └── pacejka_identification.launch.py
├── pacejka_identification/
│   ├── __init__.py
│   ├── magic_formula.py              # Pacejka formula functions
│   ├── coefficient_identifier.py     # Identification engine
│   ├── data_collector_node.py        # Standalone data collector
│   └── identification_node.py        # Main orchestration node
├── resource/
│   └── pacejka_identification
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```
