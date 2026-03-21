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

| Parameter | Default | Description |
|-----------|---------|-------------|
| `duration_seconds` | 60 | Data collection duration |
| `min_velocity` | 0.5 | Min belt velocity filter (m/s) |
| `min_fz_threshold` | 50.0 | Min normal force filter (N) |
| `method` | `dual` | `trust_region`, `differential_evolution`, `dual` |
| `axle_grouping` | `per_axle` | `per_wheel`, `per_axle`, `combined` |
| `formulas` | all 3 | `lateral_fy`, `longitudinal_fx`, `self_aligning_mz` |

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

### Trust-Region-Reflective (`trust_region`)
Local optimizer — fast but requires a good initial guess. Best for clean data.

### Differential Evolution (`differential_evolution`)
Global optimizer — no initial guess needed. Slower but robust against local minima.

### Dual (`dual`) — **Recommended**
Runs Differential Evolution first for global search, then Trust-Region for local refinement. Provides the highest accuracy, ideal for ground-truth coefficient identification.

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
