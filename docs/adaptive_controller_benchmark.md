# adaptive_controller_benchmark

Academic benchmark of `adaptive_controller_manager`'s Pure-Pursuit/MPC switching FSM (see `docs/adaptive_controller_manager.md`). This document covers what's measured, the formulas, and the literature each metric is grounded in — the package itself (`adaptive_controller_benchmark/README.md`) covers parameters, topics, and run instructions.

## 1. Why this package, and what it does not measure

`adaptive_controller_manager` arbitrates between `pure_pursuit` (bootstrap/fallback) and `mpc_path_tracking` (advanced controller) via a 6-state FSM. Two benchmark packages already exist in this repo — `tire_force_benchmark` (tire model accuracy) and `estimation_benchmark` (state-prediction accuracy) — but neither evaluates the switching *control* behavior itself: how well the arbiter tracks the raceline, how it behaves during a handover, or what a switch costs in transient tracking quality. `adaptive_controller_benchmark` fills that gap, purely as a passive subscriber to topics `adaptive_controller_manager` already publishes (`manager/state`, `manager/debug/lateral_error`, `manager/debug/heading_error`) plus `/odom`, `/drive`, `pp/drive_cmd`, `mpc/drive_cmd`, `/raceline_waypoints`, and `mpc_path_tracking`'s `/mpc/debug/solve_time_ms`. No existing controller/manager node code is modified.

## Notation

| Symbol | Description | Unit / type |
|--------|-------------|-------------|
| `e_y` | Signed lateral tracking error (from `manager/debug/lateral_error`, itself `track_geometry_utils::computeTrackError()`'s output) | m |
| `e_ψ` (heading error) | Heading tracking error (`manager/debug/heading_error`) | rad |
| `v_x` | Longitudinal velocity, from `/odom` | m/s |
| `s_m` | Per-waypoint arc-length along the raceline (`f1tenth_msgs/msg/Waypoint.s_m`) | m |
| RMS | `sqrt(mean(x_i^2))` over all logged samples of signal `x` | matches `x`'s unit |
| Max\|·\| | `max(\|x_i\|)` over all logged samples | matches `x`'s unit |
| ITAE | `Σ_i Δt_i · t_i · \|x_i\|` (rectangle-rule discretization of `∫ t·\|e(t)\| dt`, `t` measured from the accumulator's own time origin — run start for the "Overall" row) | unit·s² |
| Dwell time | Contiguous duration the FSM stays in one state before its next transition | s |
| Switching episode | One contiguous `SWITCHING_TO_MPC` or `SWITCHING_TO_PP` interval | — |
| `solve_time_ms` | MPC per-tick QP solve wall-clock time (`mpc_path_tracking`'s `/mpc/debug/solve_time_ms`) | ms |
| Lap | One full traversal of the raceline, detected via `s_m` wraparound | — |

State-name strings (`BOOTSTRAP_PP`, `RUNNING_PP`, `SWITCHING_TO_MPC`, `RUNNING_MPC`, `SWITCHING_TO_PP`, `EMERGENCY_HALT`) are matched literally against `adaptive_controller_manager/src/manager_node.cpp::fsmStateName()`'s output — a string coupling documented in `adaptive_controller_benchmark/adaptive_controller_benchmark/online_metrics.py`'s module docstring, not a shared enum.

## 2. Tracking-error metrics (RMS, Max, ITAE) — grounded in FRED-003C

Demeter, Z., Puskás, L., Kovács, B., Matkovics, Á., Nádas, M., Tuba, B., Farkas, Z., Bogár-Németh, Á., Bári, G., *"The Autonomous Software Stack of the FRED-003C: The Development That Led to Full-Scale Autonomous Racing,"* arXiv:2504.18439, 2025 — Table II of this paper compares Pure Pursuit, Stanley, and a **combined controller that adaptively selects between them per scenario** (the same problem shape as this repo's PP/MPC arbiter) using exactly:

- ITAE (Integral of Time-weighted Absolute Error)
- RMS lateral error `[m]`
- Max lateral error `[m]`
- Lap time `[s]`

`adaptive_controller_benchmark` reports the same four, split into an `Overall` row plus a `RUNNING_PP`-only and `RUNNING_MPC`-only row (`metrics_summary_table.png`), and visualizes the `RUNNING_PP` vs `RUNNING_MPC` comparison directly as a box plot (`tracking_error_boxplot_by_controller.png`) — the same controller-vs-controller comparison shape as their Table II, applied to samples of a single continuous run rather than separate runs per controller (since here one vehicle switches between both within a session).

Lap-wise comparison (rather than a single whole-run number) is standard practice in this vehicle class: Kabzan, J., de la Iglesia Valls, M., Reijgwart, V., Hendrikx, H.F.C., Ehmke, C., Prajapat, M., Bühler, A., Gosala, N., Gupta, M., Sivanesan, R., et al., *"AMZ Driverless: The Full Autonomous Racing System,"* arXiv:1905.05110, 2019 (cited by FRED-003C itself). `adaptive_controller_benchmark`'s `lap_times.png` reports lap time per completed lap for the same reason.

## 3. Switching / dwell-time metrics — grounded in switched-systems theory

Hespanha, J.P. & Morse, A.S., *"Switching between stabilizing controllers,"* Automatica, vol. 38, no. 11, pp. 1905-1917, 2002 — the seminal reference for analyzing switched-controller systems via **dwell time**: how long the system stays in one controller/mode before switching to another. A switched controller that chatters between modes with very short dwell times is a known failure mode even when each individual controller is independently stable and steady-state tracking error looks acceptable.

`SwitchTracker` (`online_metrics.py`) records, per FSM state, the list of dwell-time samples (one value per contiguous interval spent in that state), plus transition counts keyed by `(from_state, to_state)`. `metrics_summary_table.png`'s second table reports `N`/mean/max dwell time per state and the full transition-count map.

### Handover transient

`adaptive_controller_manager` documents (`docs/adaptive_controller_manager.md` §5) that during a switching episode, steering is forwarded **unmodified and instantly** from the incoming controller while speed is linearly ramped over `delta_t_switch`:

```
v_cmd(t) = (1 - α(t))·v_frozen + α(t)·v_new(t)
α(t) = clamp(t / delta_t_switch, 0, 1)
```

`adaptive_controller_benchmark` captures this empirically rather than assuming the documentation still matches the running code: each switching episode's `(t_since_switch_start, v_cmd, steering)` samples are recorded from the *actual* `/drive` topic, plotted as an overlay in `handover_transient.png` (a real-data counterpart to the existing illustrative `docs/images/handover_velocity_ramp.png`), alongside the measured `|steering_jump|` = `|entry_steering - exit_steering|` at the switch instant — a direct empirical measurement of the "instant, unmodified" steering claim, not a re-derivation of the architecture.

## 4. Compute-cost axis

MPC's per-tick solve time (`mpc_path_tracking`'s `/mpc/debug/solve_time_ms`, published when `debug.enabled && debug.log_solve_time`, both true by default — see `mpc_path_tracking/config/mpc_path_tracking.yaml`) is tracked as its own `SignalStats` accumulator (RMS/max) and plotted against the speed profile (`speed_and_compute_cost.png`). Pure Pursuit performs a small, fixed number of closed-form trigonometric operations per tick with no iterative optimization on its critical path — unlike MPC's per-tick QP solve — so no equivalent "PP solve time" topic exists to subscribe to; this asymmetry is the practical justification for benchmarking MPC's compute cost as its own axis rather than a head-to-head solve-time comparison.

## 5. Lap detection (self-derived)

No lap-completion, lap-count, or reference-speed topic exists anywhere in this repo. `f1tenth_msgs/msg/Waypoint.msg` carries a per-waypoint arc-length field `s_m`; `LapTracker` (`online_metrics.py`) tracks the nearest waypoint to the vehicle's `/odom` position each tick (brute-force `argmin` over precomputed `x_m`/`y_m` arrays — the same "brute-force, stateless nearest-point projection" approach `track_geometry_utils::computeTrackError()` already uses for the manager's own safety gates) and declares a lap boundary whenever `s_m` decreases by more than half the total track length tick-to-tick (a wraparound from near the end of the raceline back to its start). This mirrors the precedent `tire_force_benchmark` already set of re-deriving a needed signal standalone (its own vehicle-state-prediction physics) rather than assuming another node publishes it.

## 6. Safety incidents

`EMERGENCY_HALT` occurrence count and cumulative time-in-state are reported directly from `SwitchTracker`'s dwell-time bookkeeping — any occurrence at all is a notable finding for a benchmark run (see `docs/adaptive_controller_manager.md` §4, Safety 5B).

## 7. Package interfaces

See `adaptive_controller_benchmark/README.md`'s topics table and `config/benchmark_config.yaml` for the full parameter list.

## 8. How to run

See `adaptive_controller_benchmark/README.md`'s "Build"/"Run" sections, or:

```bash
ros2 launch adaptive_controller_manager adaptive_stack.launch.py \
  enable_controller_benchmark:=true \
  controller_benchmark_plot_output_dir:=/tmp/adaptive_controller_benchmark_plots
```

Plots are written when the benchmark node is shut down (`Ctrl-C` on the launch, which propagates to `destroy_node()` → `_export_plots()`), not while it's running — the same "export on shutdown" convention `tire_force_benchmark` already uses.

## References

- Demeter, Z., Puskás, L., Kovács, B., Matkovics, Á., Nádas, M., Tuba, B., Farkas, Z., Bogár-Németh, Á., Bári, G., *"The Autonomous Software Stack of the FRED-003C: The Development That Led to Full-Scale Autonomous Racing,"* arXiv:2504.18439, 2025.
- Hespanha, J.P. & Morse, A.S., *"Switching between stabilizing controllers,"* Automatica, vol. 38, no. 11, pp. 1905-1917, 2002.
- Kabzan, J., de la Iglesia Valls, M., Reijgwart, V., Hendrikx, H.F.C., Ehmke, C., Prajapat, M., Bühler, A., Gosala, N., Gupta, M., Sivanesan, R., et al., *"AMZ Driverless: The Full Autonomous Racing System,"* arXiv:1905.05110, 2019.
