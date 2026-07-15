# adaptive_controller_benchmark

ROS2 package that passively benchmarks `adaptive_controller_manager`'s PP/MPC switching FSM: tracking-error quality, switching/dwell-time behavior, handover transients, lap time, and MPC compute cost. It only subscribes to topics `adaptive_controller_manager`/`pure_pursuit`/`mpc_path_tracking` already publish — no changes to any of those packages.

See `docs/adaptive_controller_benchmark.md` (repo root `docs/`) for the metric definitions, formulas, and academic citations grounding each one.

---

## What it measures

- **Tracking error** — `e_y` (lateral) and heading error, split by active controller (`RUNNING_PP` vs `RUNNING_MPC`): RMS, max, and ITAE (integral of time-weighted absolute error) — the same metric set used by Demeter et al. 2025 (arXiv:2504.18439) to compare Pure Pursuit / Stanley / a combined switching controller.
- **Switching behavior** — dwell time per FSM state, transition counts, per-episode handover speed-ramp traces and steering-jump magnitude — grounded in Hespanha & Morse's switched-controller theory (Automatica 38(11), 2002).
- **Lap time** — derived from `/raceline_waypoints`' arc-length (`s_m`) wraparound against the nearest waypoint to `/odom`'s position; no lap-completion topic exists elsewhere in the repo, so this re-derives it standalone.
- **Compute cost** — MPC's per-solve wall-clock time (`/mpc/debug/solve_time_ms`) vs. Pure Pursuit's O(1) closed-form geometry.
- **Safety incidents** — `EMERGENCY_HALT` occurrence count and total duration.

---

## Topics subscribed (all overridable, see `config/benchmark_config.yaml`)

| Parameter | Default | Type | Source |
|---|---|---|---|
| `manager_state_topic` | `manager/state` | `std_msgs/String` | `adaptive_controller_manager` |
| `lateral_error_topic` | `manager/debug/lateral_error` | `std_msgs/Float64` | `adaptive_controller_manager` |
| `heading_error_topic` | `manager/debug/heading_error` | `std_msgs/Float64` | `adaptive_controller_manager` |
| `odom_topic` | `/odom` | `nav_msgs/Odometry` | shared |
| `drive_topic` | `/drive` | `ackermann_msgs/AckermannDriveStamped` | `adaptive_controller_manager` (arbitrated output) |
| `pp_drive_topic` | `pp/drive_cmd` | `ackermann_msgs/AckermannDriveStamped` | `pure_pursuit` (raw) |
| `mpc_drive_topic` | `mpc/drive_cmd` | `ackermann_msgs/AckermannDriveStamped` | `mpc_path_tracking` (raw) |
| `waypoint_topic` | `/raceline_waypoints` | `f1tenth_msgs/WaypointArray`, `TRANSIENT_LOCAL` | shared |
| `mpc_solve_time_topic` | `/mpc/debug/solve_time_ms` | `std_msgs/Float64` | `mpc_path_tracking` (only if its `debug.enabled`/`debug.log_solve_time` are true — both default true) |

`manager/state` drives per-tick processing (it's published every control-loop tick regardless of track-error availability); every other value is a latest-cached snapshot taken at that instant.

**FSM state-name coupling:** state strings are matched literally against `manager_node.cpp::fsmStateName()`'s output (`BOOTSTRAP_PP`, `RUNNING_PP`, `SWITCHING_TO_MPC`, `RUNNING_MPC`, `SWITCHING_TO_PP`, `EMERGENCY_HALT`) — this is a string coupling, not a shared enum. If the manager's state names ever change, update `STATE_ORDER`/`PP_ACTIVE_STATE`/`MPC_ACTIVE_STATE`/`EMERGENCY_STATE` in `adaptive_controller_benchmark_node.py` and `STATE_COLORS` in `online_metrics.py` to match.

---

## Academic plot export (on shutdown)

If `plot_output_dir` is set, the following PNGs are written when the node is closed (`Ctrl-C` → `destroy_node()`):

- `fsm_state_timeline.png` — active FSM state vs. time.
- `tracking_error_timeseries.png` — `e_y(t)`/heading-error(t), background-shaded by active state.
- `tracking_error_boxplot_by_controller.png` — `|e_y|`/`|heading error|` distributions, `RUNNING_PP` vs `RUNNING_MPC`.
- `speed_and_compute_cost.png` — `v_x(t)` vs. MPC `solve_time_ms(t)` on a twin axis.
- `handover_transient.png` — per-switch-episode speed ramp overlay + steering-jump magnitude.
- `lap_times.png` — lap-wise lap time bar chart (only written if ≥1 lap was detected).
- `metrics_summary_table.png` — tracking-error metrics table + switching/dwell/lap/compute-cost summary.

History is kept in a bounded-memory buffer (`plot_max_points`, logarithmic decimation), same strategy as `tire_force_benchmark`'s `HistoryBuffer`.

If `csv_output_path` is set, every tick is also written as a raw CSV row (`t_run_s, state, e_y_m, heading_error_rad, v_x_mps, steering_cmd_rad, speed_cmd_mps, mpc_solve_time_ms`).

---

## Build

From workspace root:

```bash
colcon build --packages-select adaptive_controller_benchmark
source install/setup.bash
```

## Run

### Standalone

```bash
ros2 launch adaptive_controller_benchmark adaptive_controller_benchmark.launch.py \
  plot_output_dir:=/tmp/adaptive_controller_benchmark_plots \
  csv_output_path:=/tmp/adaptive_controller_benchmark.csv
```

### Alongside the managed stack

```bash
ros2 launch adaptive_controller_manager adaptive_stack.launch.py \
  enable_controller_benchmark:=true \
  controller_benchmark_plot_output_dir:=/tmp/adaptive_controller_benchmark_plots \
  controller_benchmark_csv_output_path:=/tmp/adaptive_controller_benchmark.csv
```

`enable_controller_benchmark` defaults to `false` — the benchmark package is entirely inert (not even launched) unless explicitly enabled.

## Tests

Pure-logic unit tests (no rclpy needed) for `online_metrics.py`:

```bash
python3 -m pytest -p no:anyio adaptive_controller_benchmark/test/test_online_metrics.py
```

(`-p no:anyio` works around an unrelated broken `anyio` pytest-plugin/pytest version mismatch in this dev environment, not this package's own code.)

---

## File map

- Main node: `adaptive_controller_benchmark/adaptive_controller_benchmark_node.py`
- Metrics/history/switch/lap logic: `adaptive_controller_benchmark/online_metrics.py`
- Config: `config/benchmark_config.yaml`
- Launch: `launch/adaptive_controller_benchmark.launch.py`
- Tests: `test/test_online_metrics.py`
- Deeper technical writeup + citations: `docs/adaptive_controller_benchmark.md` (repo root `docs/`)
