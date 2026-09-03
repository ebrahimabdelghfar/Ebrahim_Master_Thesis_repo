# tire_force_benchmark

ROS2 package to benchmark **estimated lateral tire forces** and **estimated vehicle states** against the simulator's ground truth from `/sim/feedback/tire_forces` (`sim_manager_msgs/TireForces`) and `/odom` (`nav_msgs/Odometry`).

> **Ground truth caveat:** `/sim/feedback/tire_forces` is CARLA's own per-wheel telemetry (PhysX contact-patch forces), not physically-measured sensor data. See `docs/tire_force_benchmark.md` for details on what this does and doesn't validate.

This package supports two Fy-benchmarking modes:

1. **internal_pacejka**
   - Uses Pacejka coefficients (`C_Pf`, `C_Pr`) to estimate per-wheel lateral force $F_y$.
   - Compares internal estimate vs simulator ground truth.

2. **external_topic**
   - Subscribes to your estimator output topic (`std_msgs/Float64MultiArray`) with wheel order:
     - `[FL, FR, RL, RR]`
   - Compares your external estimates vs simulator ground truth.

In addition, it independently benchmarks a one-step-ahead **vehicle-state prediction** (lateral velocity $v_y$, yaw rate $\omega$) computed internally from the same nominal bicycle-model + Pacejka tire model, against odometry (see "Vehicle-state benchmarking" below).

---

## Configuration

All parameters live in `config/benchmark_config.yaml` (loaded by the launch file by default). A curated subset of the most commonly-tweaked keys can also be overridden on the `ros2 launch` command line (see "Run examples").

`min_fz_threshold` has **no built-in default in code** — it must be set explicitly, either in the config file or via `min_fz_threshold:=<value>` / `-p min_fz_threshold:=<value>`. A default sized for a full-size vehicle would silently reject every sample from a scaled car and produce an empty benchmark with no error (see "Bug fixes" below). Pick a value well below your vehicle's static per-wheel load (`mass * 9.81 / 4` N).

`c_pf`/`c_pr` also have **no built-in default** — benchmarking an arbitrary/hardcoded Pacejka model isn't meaningful. Provide them via `model_file` (or explicit `c_pf`/`c_pr` params) for standalone testing, or leave them unset and the node will wait for a freshly-identified model pushed via `identified_params_service` (see "Identified-params service" below) before it benchmarks anything.

---

## How it works

### Code flow chart

```mermaid
flowchart TD
  A[Start node\nTireForceBenchmarkNode] --> B[Load config_file + params\nmode, topics, filters, model/c_pf/c_pr, vehicle constants]
  B --> FE{enable_force_benchmarking}
  FE -->|true| C[Subscribe /sim/feedback/tire_forces\nsim_manager_msgs/TireForces]
  B --> Z[Create identified_params_service\nIdentifiedParam server]
  FE -->|true| D{benchmark_mode}
  B --> S{enable_state_benchmarking}

  Z -->|service call received| ZP[Split param_values 0:4/4:8\ninto c_pf/c_pr, set have_identified_params=true]
  ZP -.unblocks.-> E
  ZP -.unblocks.-> U

  D -->|internal_pacejka| E[On tire_forces callback\nvalidate sample + have_identified_params?]
  E --> F[Compute Fy_est per wheel\nPacejka(C_Pf/C_Pr, slip_angle, Fz)]
  F --> G[Benchmark GT Fy vs Est Fy\nFL FR RL RR + axle sums + total]
  G --> H[Publish /benchmarking/tire_force_fy_estimate\nFloat64MultiArray]
  G --> I[Update online metrics\nRMSE MAE NRMSE MaxAE Bias Std R2]
  I --> J{sample_count % log_interval == 0}
  J -->|yes| K[Publish /benchmarking/tire_force_summary\nString + log to terminal]
  J -->|no| L[Wait next sample]

  D -->|external_topic| M[Subscribe estimated_fy_topic\nFloat64MultiArray [FL FR RL RR]]
  C --> N[On tire_forces callback\nvalidate + queue GT Fy]
  M --> O[On external estimate callback\ncheck len(data)>=4, queue estimate]
  N --> O
  O --> P[Slide GT queue by 1 per estimate\npair estimate[k] with ground_truth[k+lead]]
  P --> I

  S -->|true, model constants set| T[Subscribe odom_topic + ackermann_cmd_topic]
  T --> U[On odom callback\nhave_identified_params? + one-step bicycle+Pacejka prediction\nv_y_pred, omega_pred vs real v_y, omega]
  U --> V[Publish state_estimate/state_sensor/state_error\nUpdate v_y/omega online metrics]
  V --> I

  I --> Q{plot_output_dir set?}
  Q -->|yes, on shutdown| R[Export PNG plots\nforces + states timeseries/histograms/summary table]
  I --> W{csv_output_path set?}
  W -->|yes| X[Write CSV row\nGT/Est per wheel + sums, and states CSV]
```

### Ground truth input

- `/sim/feedback/tire_forces` (`sim_manager_msgs/TireForces`), `float64[4]` arrays in `wheel_names` order (`FL, FR, RL, RR`): `lateral_force` (Fy GT), `normal_load` (Fz), `slip_angle`. `lateral_force` is already sign-flipped into the ROS body frame to match `slip_angle`, so it compares directly against the Pacejka evaluation.
- `/odom` (`nav_msgs/Odometry`): `twist.twist.linear.x/y`, `twist.twist.angular.z` — used as $v_x, v_y, \omega$ ground truth for state benchmarking.
- `/drive` (`ackermann_msgs/AckermannDriveStamped`): `drive.steering_angle` — used as $\delta$.

### Valid Fy sample filtering

Each tire-forces sample is used only if:

- `|normal_load| >= min_fz_threshold` on all four wheels
- `slip_angle` and `normal_load` are valid numbers
- the frame is not a standstill frame: below 0.5 m/s the publisher zeroes `slip_angle` and both forces while `normal_load` stays live, so an all-zero slip/Fy frame is dropped

### Fy estimation path by mode

- `internal_pacejka`: computes per-wheel estimated $F_y$ from slip angle and normal load using the Pacejka Magic Formula (see `docs/tire_force_benchmark.md` for the equation and citation). **Does nothing until `have_identified_params` is true** (see "Identified-params service" below) — no metrics are updated and no estimate is published while waiting.
- `external_topic`: compares your external estimated array `[FL, FR, RL, RR]` against ground truth; unaffected by `have_identified_params` since it doesn't use `c_pf`/`c_pr` for Fy (vehicle-state benchmarking still does, see below).

### Identified-params service

The node has no hardcoded Pacejka model. `c_pf`/`c_pr` (and, if not already provided by `model_file`, the vehicle constants) come from one of:

1. `model_file` at startup (a `C_Pf`/`C_Pr` (+ optionally `m`/`I_z`/`l_f`/`l_r`/`l_wb`) yaml), or
2. explicit `c_pf`/`c_pr` parameters at startup, or
3. a call to `identified_params_service` (default `/benchmark/update_params`, `adaptive_controller_interfaces/srv/IdentifiedParam`) — `request.param_values` is 8 floats `[Bf,Cf,Df,Ef,Br,Cr,Dr,Er]`, split into `c_pf = param_values[0:4]`, `c_pr = param_values[4:8]`. Acks `true` on success (exactly 8 values), `false` otherwise.

Until one of these has happened at least once (`have_identified_params`), `internal_pacejka` Fy benchmarking and vehicle-state prediction are both held off entirely — benchmarking an arbitrary/never-identified model isn't meaningful. Later service calls (e.g. from periodic re-identification) update `c_pf`/`c_pr` live, mid-run.

`adaptive_controller_manager` can push its own accepted `sysid/update_params` submissions here automatically — set `benchmark_update_params_enable: true` in its config (see `docs/adaptive_controller_manager.md`). Match `identified_params_service` here to its `benchmark_update_params_service` (both default to `benchmark/update_params`).

### Robust t+lead alignment (external mode)

Ground truth and external estimates are each buffered in a queue. Pairing rule:

- compare `estimate[k]` with `ground_truth[k + lead]`, where `lead = external_prediction_lead_samples`
- after each pairing, the ground-truth queue window slides forward by exactly **one** sample (previously it incorrectly slid by `lead + 1`, drifting the alignment further apart every iteration for `lead >= 1` — fixed, see `docs/tire_force_benchmark.md`)

Set `external_prediction_lead_samples:=1` for a typical $t \rightarrow t+1$ predictor.

### Vehicle-state benchmarking ($v_y$, $\omega$)

Independently of the Fy benchmarking above, if `enable_state_benchmarking` is true, vehicle constants (`m`, `I_z`, `l_f`, `l_r`, `l_wb`) are available (from `model_file` or explicit parameters), and `have_identified_params` is true (see "Identified-params service" above), the node:

1. Subscribes to `odom_topic` and `ackermann_cmd_topic`.
2. On each new odometry sample, computes a one-step-ahead prediction of $v_y, \omega$ from the *previous* real state using the same bicycle-model + Pacejka tire forces (front/rear slip angles, `pacejka_formula`, lateral/yaw dynamics, Euler-integrated over the real inter-sample `dt`), reusing the same math already validated in `On-Track-SysID/src/on_track_sys_id.py`.
3. Compares the prediction against the *current* real $v_y, \omega$ (one-step-ahead delayed comparison), updates `v_y`/`omega` online metrics, and publishes `state_estimate_topic`/`state_sensor_topic`/`state_error_topic`.

Predictions are skipped (previous state just re-cached) when the timestep is non-monotonic, when the inter-sample gap is too large to trust an Euler step (`dt > 0.2s`), or while the car is nearly stopped (`v_x < 0.1 m/s`, since the slip-angle formula divides by $v_x$).

If `m`/`I_z`/`l_f`/`l_r`/`l_wb` aren't available, state benchmarking is disabled with a warning rather than the node crashing.

### Metrics computed online

For each signal, metrics are updated incrementally (O(1) per sample): RMSE, MAE, NRMSE, MaxAE, Bias, StdDev, R².

Signals benchmarked: FL/FR/RL/RR Fy, front/rear axle Fy sum, total vehicle Fy sum, $v_y$, $\omega$.

### Academic plot export

If `plot_output_dir` is set, on node shutdown the following PNGs are written (matplotlib, publication-style — serif font, labeled axes with units, grid, legend on every figure, 300 DPI):

- `tire_forces_timeseries.png` — ground truth vs estimate, one subplot per Fy signal.
- `vehicle_states_timeseries.png` — ground truth vs estimate for $v_y$, $\omega$.
- `tire_forces_error_hist.png` / `vehicle_states_error_hist.png` — error distribution histograms, with the zero-error line and mean bias marked.
- `tire_forces_parity.png` / `vehicle_states_parity.png` — estimate-vs-ground-truth parity (regression) scatter with a y=x reference line and RMSE/R² annotated, one subplot per signal — the standard model-validation figure in this field (cf. Dikici et al. 2024, Fig. 5/6-style validation).
- `pacejka_curve_validation.png` — measured Fy vs. slip angle scatter overlaid with the identified Magic Formula curve, front/rear axle side by side at their nominal static loads (`internal_pacejka` mode only, once a model has been identified) — the canonical Pacejka-model validation plot (Bakker/Nyborg/Pacejka SAE 870421; Pacejka & Bakker 1992).
- `pacejka_identified_vs_nominal.png` — the identified model vs. a nominal/prior reference model (`nominal_model_file`), swept analytically over a fixed slip-angle range, front/rear stacked — "how much did identification actually change the model," requires both a live identified model and `nominal_model_file` set.
- `metrics_summary.png` — a table of RMSE/MAE/NRMSE/MaxAE/Bias/StdDev/R² for every signal.

History is kept in a bounded-memory buffer (`plot_max_points`, logarithmic decimation) so long runs don't grow memory unbounded.

---

## Topics

### Subscriptions

- `/sim/feedback/tire_forces` (`sim_manager_msgs/TireForces`) — only if `enable_force_benchmarking`
- `estimated_fy_topic` (`std_msgs/Float64MultiArray`) — only if `enable_force_benchmarking` and `external_topic` mode
- `odom_topic` (`nav_msgs/Odometry`) — only if `enable_state_benchmarking`
- `ackermann_cmd_topic` (`ackermann_msgs/AckermannDriveStamped`) — only if `enable_state_benchmarking`

### Services

- `identified_params_service` (default `/benchmark/update_params`, server, `adaptive_controller_interfaces/srv/IdentifiedParam`) — see "Identified-params service" above.

### Publications

- `/benchmarking/tire_force_fy_estimate` (`std_msgs/Float64MultiArray`, `internal_pacejka` mode only) — `[FL, FR, RL, RR, FrontSum, RearSum, TotalSum]`
- `/benchmarking/tire_force_summary` (`std_msgs/String`) — human-readable summary of all metrics (Fy + states) every `log_interval` samples
- `state_estimate_topic` (default `/benchmarking/state_estimate`, `[v_x, v_y_pred, omega_pred]`)
- `state_sensor_topic` (default `/benchmarking/state_sensor`, `[v_x, v_y_real, omega_real, delta]`)
- `state_error_topic` (default `/benchmarking/state_error`, `[|err_v_y|, |err_omega|]`)

---

## Parameters

See `config/benchmark_config.yaml` for the full, documented, canonical list. Summary:

| Parameter | Default | Notes |
|---|---|---|
| `enable_force_benchmarking` | `true` | false: no tire-forces/`estimated_fy_topic` subscription at all, no Fy metrics |
| `benchmark_mode` | `internal_pacejka` | `internal_pacejka` or `external_topic` |
| `tire_forces_topic` | `/sim/feedback/tire_forces` | `sim_manager_msgs/TireForces`, from the CARLA bridge workspace |
| `estimated_fy_topic` | `/estimated_tire_force_fy` | `external_topic` only |
| `external_prediction_lead_samples` | `1` | `external_topic` only |
| `external_max_queue_size` | `2000` | `external_topic` only |
| `model_file` | `''` | `C_Pf`/`C_Pr` (+ optionally `m`/`I_z`/`l_f`/`l_r`/`l_wb`) |
| `nominal_model_file` | `''` | reference `C_Pf`/`C_Pr` for `pacejka_identified_vs_nominal.png` only — never seeds the live benchmark model |
| `c_pf` / `c_pr` | **none** | optional startup override; otherwise wait for `identified_params_service` |
| `identified_params_service` | `/benchmark/update_params` | `adaptive_controller_interfaces/srv/IdentifiedParam` server |
| `min_fz_threshold` | **none — mandatory** | see "Configuration" above |
| `log_interval` | `200` | |
| `csv_output_path` | `''` | also writes a sibling `*_states.csv` if state benchmarking is enabled |
| `enable_state_benchmarking` | `true` | auto-disables with a warning if vehicle constants are missing |
| `odom_topic` | `/odom` | |
| `ackermann_cmd_topic` | `/drive` | |
| `state_estimate_topic` / `state_sensor_topic` / `state_error_topic` | see yaml | |
| `m` / `I_z` / `l_f` / `l_r` / `l_wb` | `0.0` | vehicle constants, fallback if not in `model_file` |
| `plot_output_dir` | `''` | set to enable PNG export on shutdown |
| `plot_max_points` | `5000` | history buffer size per signal before decimation |

---

## Build

From workspace root:

```bash
colcon build --packages-select tire_force_benchmark
source install/setup.bash
```

---

## Run examples

### 1) Default config file (recommended)

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py
```

Uses `config/benchmark_config.yaml` as shipped — edit that file (or pass `config_file:=/path/to/your.yaml`) to point `model_file` at a real vehicle model and set `min_fz_threshold`/vehicle constants for your car.

### 2) Internal Pacejka mode with a specific model file

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  benchmark_mode:=internal_pacejka \
  model_file:=/home/ebrahim/Ebrahim_Master_Thesis_repo/On-Track-SysID/models/SIM/SIM_pacejka.txt
```

### 3) External estimator mode

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  benchmark_mode:=external_topic \
  estimated_fy_topic:=/your_estimated_tire_force_topic \
  external_prediction_lead_samples:=1
```

### 4) Enable CSV logging and plot export

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  csv_output_path:=/tmp/tire_force_benchmark.csv \
  plot_output_dir:=/tmp/tire_force_benchmark_plots
```

### 5) Custom config file

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py \
  config_file:=/path/to/my_benchmark_config.yaml
```

### 6) Wait for adaptive_controller_manager's identified params (no model_file/c_pf/c_pr)

```bash
ros2 launch tire_force_benchmark tire_force_benchmark.launch.py
# ... elsewhere, with adaptive_controller_manager's benchmark_update_params_enable: true ...
# the node logs "waiting for identified params via service" until On-Track-SysID's
# first identification completes and the manager forwards it here automatically.
```

To push params manually (e.g. for testing without the manager running):
```bash
ros2 service call /benchmark/update_params adaptive_controller_interfaces/srv/IdentifiedParam \
  "{param_values: [6.63, 1.1052, 0.4316, 0.5193, 7.8594, 1.5468, 0.3589, 0.5631]}"
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
- Online metrics + history buffer: `tire_force_benchmark/online_metrics.py`
- Config: `config/benchmark_config.yaml`
- Launch: `launch/tire_force_benchmark.launch.py`
- Tests: `test/test_queue_alignment.py`
- Deeper technical writeup + citations: `docs/tire_force_benchmark.md` (repo root `docs/`)
