# tire_force_benchmark

ROS2 package to benchmark **estimated lateral tire forces** against IPG CarMaker ground truth from `/tire_forces` (`hellocm_msgs/TireForcesArray`).

This package supports two benchmarking modes:

1. **internal_pacejka**
   - Uses Pacejka coefficients (`C_Pf`, `C_Pr`) to estimate per-wheel lateral force $F_y$.
   - Compares internal estimate vs CarMaker ground truth.

2. **external_topic**
   - Subscribes to your estimator output topic (`std_msgs/Float64MultiArray`) with wheel order:
     - `[FL, FR, RL, RR]`
   - Compares your external estimates vs CarMaker ground truth.

---

## How it works

### Code flow chart

```mermaid
flowchart TD
  A[Start node\nTireForceBenchmarkNode] --> B[Load params\nmode, topics, filters, model/c_pf/c_pr]
  B --> C[Subscribe /tire_forces\nhellocm_msgs/TireForcesArray]
  B --> D{benchmark_mode}

  D -->|internal_pacejka| E[On /tire_forces callback\nvalidate sample]
  E --> F[Compute Fy_est per wheel\nPacejka(C_Pf/C_Pr, slip_angle, Fz)]
  F --> G[Benchmark GT Fy vs Est Fy\nFL FR RL RR + axle sums + total]
  G --> H[Publish /benchmarking/tire_force_fy_estimate\nFloat64MultiArray]
  G --> I[Update online metrics\nRMSE MAE NRMSE MaxAE Bias Std R2]
  I --> J{sample_count % log_interval == 0}
  J -->|yes| K[Publish /benchmarking/tire_force_summary\nString + log to terminal]
  J -->|no| L[Wait next sample]

  D -->|external_topic| M[Subscribe estimated_fy_topic\nFloat64MultiArray [FL FR RL RR]]
  C --> N[On /tire_forces callback\nvalidate + cache latest GT Fy]
  M --> O[On external estimate callback\ncheck len(data)>=4]
  N --> O
  O --> P[Benchmark cached GT Fy vs external Est Fy\nFL FR RL RR + axle sums + total]
  P --> I

  I --> Q{csv_output_path set?}
  Q -->|yes| R[Write CSV row\nGT/Est per wheel + sums]
  Q -->|no| L
  R --> L
```

### Ground truth input

- Subscribes to `/tire_forces` (`hellocm_msgs/TireForcesArray`), which includes for each wheel:
  - `fy` (lateral force ground truth)
  - `fz` (normal load)
  - `slip_angle`
  - `on_road`

### Valid sample filtering

Each sample is used only if:

- `on_road == true` (when `require_on_road` is enabled)
- `|fz| >= min_fz_threshold`
- `slip_angle` and `fz` are valid numbers

### Estimation path by mode

- `internal_pacejka`:
  - Computes per-wheel estimated $F_y$ from slip angle and normal load using Pacejka formula.
- `external_topic`:
  - Uses latest ground truth sample and compares against external estimated array `[FL, FR, RL, RR]`.

### Robust t+1 alignment (external mode)

For one-step-ahead estimators, this package now uses a **queue-based alignment** in `external_topic` mode.

- Ground truth samples are buffered from `/tire_forces`.
- External estimates are buffered from `estimated_fy_topic`.
- Pairing rule uses sample index shift:
  - compare `estimate[k]` with `ground_truth[k + lead]`
  - where `lead = external_prediction_lead_samples`

Set `external_prediction_lead_samples:=1` for a typical $t \rightarrow t+1$ predictor.

To avoid data loss under bursty timing, queue size is configurable via `external_max_queue_size`.

### Metrics computed online

For each signal, metrics are updated incrementally (O(1) per sample):

- RMSE
- MAE
- NRMSE
- MaxAE
- Bias
- StdDev
- R²

Signals benchmarked:

- FL Fy
- FR Fy
- RL Fy
- RR Fy
- Front axle Fy sum
- Rear axle Fy sum
- Total vehicle Fy sum

---

## Topics

### Subscriptions

- `/tire_forces` (`hellocm_msgs/TireForcesArray`) — required ground truth
- `/estimated_tire_force_fy` (`std_msgs/Float64MultiArray`) — only in `external_topic` mode

### Publications

- `/benchmarking/tire_force_fy_estimate` (`std_msgs/Float64MultiArray`)
  - Only in `internal_pacejka` mode
  - Data format: `[FL, FR, RL, RR, FrontSum, RearSum, TotalSum]`

- `/benchmarking/tire_force_summary` (`std_msgs/String`)
  - Human-readable summary of all metrics every `log_interval` samples

---

## Parameters

- `benchmark_mode` (string, default: `internal_pacejka`)
  - `internal_pacejka` or `external_topic`

- `tire_forces_topic` (string, default: `/tire_forces`)

- `estimated_fy_topic` (string, default: `/estimated_tire_force_fy`)
  - Used only in `external_topic` mode

- `external_prediction_lead_samples` (int, default: `1`)
  - Used only in `external_topic`
  - Robust index shift for prediction horizon (set to `1` for one-step-ahead estimators)

- `external_max_queue_size` (int, default: `2000`)
  - Used only in `external_topic`
  - Maximum buffer length for GT and estimate queues

- `model_file` (string, default: empty)
  - Path to model file containing `C_Pf` and `C_Pr` arrays
  - Used in `internal_pacejka` mode

- `c_pf` (double[4], default provided)
- `c_pr` (double[4], default provided)
  - Fallback Pacejka coefficients when `model_file` is not provided/invalid

- `min_fz_threshold` (double, default: `50.0`)

- `require_on_road` (bool, default: `true`)

- `log_interval` (int, default: `200`)

- `csv_output_path` (string, default: empty)
  - If set, writes per-sample GT/estimate data to CSV

---

## Build

From workspace root:

```bash
colcon build --packages-select tire_force_benchmark
source install/setup.bash
```

---

## Run examples

### 1) Internal Pacejka mode

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  benchmark_mode:=internal_pacejka \
  model_file:=/home/ebrahim/Ebrahim_Master_Thesis_repo/On-Track-SysID/models/SIM/SIM_pacejka.txt
```

### 2) External estimator mode

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  benchmark_mode:=external_topic \
  estimated_fy_topic:=/your_estimated_tire_force_topic \
  external_prediction_lead_samples:=1
```

### 3) Enable CSV logging

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  benchmark_mode:=external_topic \
  estimated_fy_topic:=/your_estimated_tire_force_topic \
  csv_output_path:=/tmp/tire_force_benchmark.csv
```

---

## External topic contract (`external_topic` mode)

Your estimator must publish:

- Topic type: `std_msgs/Float64MultiArray`
- `data` length: at least 4
- Wheel order: `[FL, FR, RL, RR]`
- Units: Newton (N), lateral force $F_y$

If less than 4 values are received, the sample is ignored.

---

## File map

- Main node: `tire_force_benchmark/tire_force_benchmark_node.py`
- Online metrics: `tire_force_benchmark/online_metrics.py`
- Launch: `launch/tire_force_benchmark.launch.py`
