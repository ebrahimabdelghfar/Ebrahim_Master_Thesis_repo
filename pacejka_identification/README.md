# Pacejka Magic Formula Coefficient Identification

## Overview

This ROS2 package performs **direct identification** of Pacejka Magic Formula tire model coefficients using ground-truth tire force data from the CARLA bridge, published on `/sim/feedback/tire_forces` (`sim_manager_msgs/TireForces`).

Every field of that message is CARLA's own `WheelTelemetryData` — slip angle, slip ratio, normal load, lateral force, effective friction coefficient and wheel angular velocity straight out of the simulator, with no analytic tire model in between — so the coefficients fitted here cannot disagree with the physics the vehicle is actually driven by. They serve as **ground truth** for benchmarking other identification methods (e.g. the indirect NN-based approach in `On-Track-SysID`).

The message type lives in the CARLA bridge workspace, so source it *after* this workspace's overlay:

```bash
source ~/Ebrahim_Master_Thesis_repo/install/setup.bash
source ~/Carla_ASU_Bridge/install/ros_apps/setup.bash
```

### What the source message can and cannot identify

| Channel | Available? | Why |
|---|---|---|
| `lateral_fy` | **Yes — ground truth** | `lateral_force` is a validated contact-patch force (sum tracks `m·a_y`, corr +0.95, slope 0.95–1.08), already sign-flipped into the ROS body frame to match `slip_angle`. |
| `longitudinal_fx` | Selectable, **not** ground truth | `longitudinal_force` is not the force the chassis receives: it behaves as a friction-capacity report, sitting above `0.80 · tire_friction · normal_load` half the time and exactly on that limit 15 % of the time, while `corr(sum, m·a_x)` is +0.04. A Pacejka fit against it recovers the capacity envelope, not a tire curve. `wheel_torque` is exactly `-longitudinal_force · wheel_radius`, so it adds nothing. Off by default; the node warns and reports the saturated fraction if you enable it. |
| `self_aligning_mz` | **No** | `TireForces` carries no self-aligning torque. Requesting it logs an error and drops it. |
| `tire_friction` | Cross-check | The effective μ the physics step uses (the configured wheel friction already multiplied by the road surface — 1.05 published against 1.5 configured). The node logs it per group and warns when the identified `D` exceeds it by more than 5 %. |

### Magic Formula

The standard Pacejka "Magic Formula" for a single force/torque channel:

```
Y(x) = Fz · D · sin(C · arctan(B·x − E·(B·x − arctan(B·x))))
```

Where:
- **x** = slip variable (slip angle α for Fy, slip ratio κ for Fx)
- **Fz** = vertical (normal) load
- **B** = stiffness factor
- **C** = shape factor
- **D** = peak factor
- **E** = curvature factor

## Features

| Feature | Description |
|---------|-------------|
| 2 force channels | Fy (lateral, ground truth) and Fx (longitudinal, drivetrain effort — see above) |
| 8 identification methods | Trust-Region, Differential Evolution, Dual (DE→TR), GA, GA→TR, JADE (adaptive DE), JADE→TR, Bayesian SVI |
| 3 grouping modes | per_wheel, per_axle, combined |
| MAP regularization | Literature bounds + Gaussian prior on C (simultaneous mode) |
| Data rebalancing | Optional slip-bin subsampling to avoid overfitting the linear region |
| Automatic CSV export | Raw dataset with per-wheel columns |
| Automatic YAML export | Identified [B,C,D,E] (+ uncertainty for `bayesian_svi`) with fit metrics (R², RMSE) |
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

The launch file takes no arguments — every setting is read from
`config/identification_config.yaml`. Edit that file, rebuild, then run:

```bash
ros2 launch pacejka_identification pacejka_identification.launch.py
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
| `duration_seconds` | 60 | Collection window, in seconds of wall clock from the first message (the topic rate is not fixed, so samples are not counted) |
| `odom_topic` | `/odom` | Speed source for the standstill gate (`""` disables the gate) |
| `min_speed` | 2.0 | Reject samples below this speed (m/s); `0.0` disables the gate. Replaces the old `min_velocity` — `TireForces` has no per-wheel belt velocity |
| `min_fz_threshold` | 50.0 | Min normal force filter (N) |
| `method` | `dual` | See **Identification Methods** below |
| `axle_grouping` | `per_axle` | `per_wheel`, `per_axle`, `combined` |
| `formulas` | `[lateral_fy]` | `lateral_fy`, `longitudinal_fx` |

The standstill gate matters. The publisher itself zeroes `slip_angle`, `slip_ratio`,
`lateral_force`, `longitudinal_force` and `wheel_torque` below 0.5 m/s — PhysX sleeps a
parked vehicle and the telemetry then repeats byte for byte — while `normal_load`,
`tire_friction` and `wheel_speed` stay live, so the `min_fz_threshold` filter alone would
let those empty samples through. Both nodes drop them unconditionally, and `min_speed`
raises the floor further: a few thousand near-zero-slip samples pin the fit to the linear
region and make `D` meaningless. The node also logs `p99|alpha|` and the peak `|Fy|/Fz` per group,
and warns when slip stays under 0.03 rad — below that only the product `B·C·D` is
identifiable, so a reported `D` is not a friction measurement.

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
| `/sim/feedback/tire_forces` | `sim_manager_msgs/TireForces` | Ground-truth per-wheel tire telemetry from CARLA |
| `/odom` | `nav_msgs/Odometry` | Speed for the standstill gate (only if `min_speed > 0`) |

## Output Files

After identification completes, two files are exported to `~/`:

1. **CSV** (`pacejka_id_data_<timestamp>.csv`) — raw dataset
2. **YAML** (`pacejka_id_coefficients_<timestamp>.yaml`) — identified coefficients with metrics

### Example YAML Output

```yaml
identification:
  method: dual
  formulas: [lateral_fy]
  axle_grouping: per_axle
  source_topic: /sim/feedback/tire_forces
  source_msg: sim_manager_msgs/TireForces
  messages_received: 5981
  messages_kept: 5402
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

The `method` parameter allows you to select the backend optimization solver. The sequential mode fixes D — the friction coefficient μ — to the measured peak (breaking the dominant part of the Magic Formula's B-C-D-E parameter degeneracy) and these solvers fit B, C and E:

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

### 8. Bayesian SVI (`bayesian_svi`)
Stochastic Variational Inference (Pyro): fits a correlated multivariate-normal
posterior over the free parameters against the ELBO objective, following
Goblirsch et al., "Bayesian Optimization-based Tire Parameter and
Uncertainty Estimation for Real-World Data" (TUM, arXiv:2504.20863, 2025).
Returns a point estimate (posterior mean) plus `*_std` uncertainty for each
parameter — useful when the ground-truth fit's confidence matters, not just
its curve-fit quality. Slower than the other methods; requires the optional
`pyro-ppl` dependency.

## Anti-Overfitting Options (Simultaneous Mode)

Simultaneous mode has no fixed parameters, so it's the most exposed to the
Magic Formula's B-C-E degeneracy and to overfitting an unevenly-sampled slip
range. Two independent, opt-in mechanisms address this (both selectable via
config, and combinable):

- **`identification.regularization: "map"`** — switches to literature-consistent
  bounds and adds a Gaussian-prior penalty anchoring C toward its literature
  value (see `map_regularization` in the config). Classical MAP estimate, no
  new dependency.
- **`identification.method: "bayesian_svi"`** — full Bayesian treatment (see
  above), usable in both sequential and simultaneous mode.
- **`data_balancing.enabled: true`** — rebalances the pooled slip samples into
  equal-width bins before fitting, so a dense near-zero-slip cluster (typical
  of real driving data) doesn't dominate the fit and starve the sparser
  near-peak region of influence.

## Architecture

```
identification_node
    │
    ├── Phase 1: Data Collection
    │   └── Subscribe to /sim/feedback/tire_forces → buffer per-wheel arrays
    │       (gated on /odom speed and |Fz|, for duration_seconds of wall clock)
    │
    ├── Phase 2: Identification
    │   └── CoefficientIdentifier.identify_fy/fx()
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
