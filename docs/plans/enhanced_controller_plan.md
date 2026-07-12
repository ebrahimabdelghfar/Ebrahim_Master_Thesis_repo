# Overview

Design and implement the adaptive controller stack for the Ackermann-steered racecar: [Pure Pursuit](pure_pursuit/) as the bootstrap/fallback controller, [mpc_path_tracking](mpc_path_tracking/) as the advanced controller once tire parameters are identified, mediated by a new arbiter node, `adaptive_controller_manager`.

**Grounded against the current repo state** (this section exists so nothing below is designed against a stale assumption):

- `mpc_path_tracking` already exists and is **C++ / OSQP** (`osqp-eigen`, vendored via `CMakeLists.txt` FetchContent), not `do-mpc`/CasADi and not acados. `thirdparty_lib/acados` is vendored but wired into nothing — dead weight, out of scope here. `config/mpc_path_tracking.yaml:69` even flags `backend: "osqp"  # stage 1: osqp; stage 2 option: acados` — acados is a future option, not a current dependency.
- `mpc_path_tracking` already publishes a health signal: `/mpc/status` (`diagnostic_msgs/msg/DiagnosticStatus`, `debug_publisher.cpp`), `level == OK` iff the QP solved. It already computes lateral error `e_y` and heading error `e_psi` every tick (`mpc_node.cpp:150-156`) via nearest-point projection onto `/raceline_waypoints`, but only *publishes* them (`/mpc/debug/lateral_error`, `/mpc/debug/heading_error`) when `debug.enabled: true`, and obviously only while the MPC node itself is running.
- `pure_pursuit` has **no** enable/disable input, health output, or state output today — it publishes `drive` unconditionally every tick.
- `On-Track-SysID` has **no** `First_run`, `estimated_param`, or service interface today. Its only external signal is a one-shot latched `/sysid/training_complete` (`std_msgs/String`, YAML-encoded `{C_Pf, C_Pr, racecar_version}`), published once, never repeated — continuous re-identification does not exist yet.
- Both `pure_pursuit` and `mpc_path_tracking` currently publish their `drive` output **directly** to `/drive` — there is no arbiter today. `f1tenth_simulator/node/mux.cpp` is working prior art for exactly this pattern (private per-source topic in, single arbitrated topic out, hardcoded zero-command failsafe when nothing is active) and this plan mirrors it.
- SysID's identified tire parameters (`C_Pf`, `C_Pr`, each a 4-element Pacejka `[B, C, D, E]`, see `On-Track-SysID/models/SIM/SIM_pacejka.txt` and `On-Track-SysID/params/pacejka_params.yaml`) map 1:1 onto `mpc_path_tracking`'s tire config block (`tire.{Bf,Cf,Df,Ef,Br,Cr,Dr,Er}`, `config/mpc_path_tracking.yaml:44-52`). No unit/shape mismatch — good, the parameter service can be a flat `float32[8]` in the fixed order `[Bf, Cf, Df, Ef, Br, Cr, Dr, Er]`.

## Design decisions (resolved ambiguities from the original draft)

1. **Handover blending.** The original draft described two conflicting mechanisms (a velocity-ramp profile in the FSM section vs. a full `u(t) = α·u_mpc + (1-α)·u_pp` blend in the safety section) — the latter implies both controllers running concurrently, which contradicts the strict mutually-exclusive `Start_Working_pp`/`Start_Working_mpc` gating used everywhere else. Resolved as: **steering switches instantly, speed is ramped**. At the switch instant, the manager freezes the outgoing controller's last commanded speed (`v_frozen`) and immediately deactivates it (`Start_Working_pp/mpc = false`, single-writer from then on). Only the newly active controller runs from that point forward; its raw steering command is forwarded unmodified, while its speed command is blended: `v_cmd(t) = (1-α(t))·v_frozen + α(t)·v_new(t)`, with `α(t)` ramping 0→1 linearly over `Δt_switch`. No dual-controller concurrent execution required.
2. **Error signal source.** `e_y`/`e_psi` must be available continuously — including while Pure Pursuit is active and MPC hasn't started — to gate arming MPC. Resolved as: **extract the nearest-point-projection math into a shared library**, new package `track_geometry_utils`, consumed by both `mpc_path_tracking` (refactored to call it instead of its private copy) and `adaptive_controller_manager`. Single source of truth, no duplicated math, no dependency on MPC's debug topics being enabled.
3. **MPC health signal.** Resolved as: **reuse `/mpc/status`** as-is. The manager subscribes `diagnostic_msgs/msg/DiagnosticStatus` directly and derives `mpc_health := (status.level == DiagnosticStatus::OK)` (see `debug_publisher.cpp:66-68` — `level` already IS the solved/not-solved signal; no new topic needed in `mpc_path_tracking`). `pure_pursuit` gets new, plain `std_msgs/msg/Bool` topics (`pp_state`, `pp_health`) since it has nothing to reuse.

# Target Packages

## 1. `adaptive_controller_interfaces` (new, `ament_cmake`, interfaces-only)

One service definition (see below). ROS2 interface files must be PascalCase — the original draft's `Identified_param_service.srv` is invalid; use `IdentifiedParam.srv`.

## 2. `track_geometry_utils` (new, `ament_cmake`, header-only or small static lib)

- Extract the projection currently inlined in `mpc_path_tracking/src/mpc_node.cpp:150-156` into a free function, e.g.:
  ```cpp
  namespace track_geometry_utils {
  struct TrackError { double e_y; double heading_error; size_t nearest_idx; };
  TrackError computeTrackError(
    const f1tenth_msgs::msg::WaypointArray & waypoints,
    double x, double y, double yaw);
  }
  ```
- `mpc_path_tracking` depends on this package and calls it instead of its private copy (behavior-preserving refactor — same nearest-point algorithm, same output values).
- `adaptive_controller_manager` depends on it too, feeding it `/odom` + `/raceline_waypoints` directly, independent of whether MPC is running.

## 3. `adaptive_controller_manager` (new, C++, `ament_cmake`)

**Purpose:** owns the FSM, arbitrates the single `/drive` output (mirrors `f1tenth_simulator/node/mux.cpp`'s pattern: private per-controller input topics in, one shared output topic out, hardcoded zero-command failsafe when nothing is active), mediates parameter handoff between SysID and MPC, monitors health.

**Node:** `adaptive_controller_manager_node`, single-threaded control-loop logic driven by a timer at the vehicle's control rate, but registered on a `rclcpp::executors::MultiThreadedExecutor` with the parameter-service-client callback on its own `ReentrantCallbackGroup` — parameter forwarding to MPC must not block the arbitration timer (see workflow step 3).

**Subscribes:**
| Topic | Type | From |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | simulator/vehicle |
| `/raceline_waypoints` | `f1tenth_msgs/msg/WaypointArray` (`transient_local`) | map loader |
| `pp/drive_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | `pure_pursuit` (renamed from `/drive`) |
| `mpc/drive_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | `mpc_path_tracking` (renamed from `/drive`) |
| `pp_state`, `pp_health` | `std_msgs/msg/Bool` | `pure_pursuit` (new) |
| `/mpc/status` | `diagnostic_msgs/msg/DiagnosticStatus` | `mpc_path_tracking` (existing, reused as `mpc_health`) |
| `sysid/first_run` | `std_msgs/msg/Bool` | `On-Track-SysID` (new) |

**Publishes:**
| Topic | Type | To |
|---|---|---|
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | vehicle (sole writer) |
| `Start_Working_pp` | `std_msgs/msg/Bool` | `pure_pursuit` |
| `Start_Working_mpc` | `std_msgs/msg/Bool` | `mpc_path_tracking` |

**Services:**
| Name | Type | Role |
|---|---|---|
| `sysid/update_params` | `adaptive_controller_interfaces/srv/IdentifiedParam` | server — `On-Track-SysID` calls this to submit new tire params |
| `mpc/update_params` | `adaptive_controller_interfaces/srv/IdentifiedParam` | client — manager forwards validated params to `mpc_path_tracking` |

The manager is the middle validator: SysID submits candidate params to the manager's server; the manager range-checks them, then (only if arming/re-arming conditions allow) forwards them via its own client call to MPC's server; MPC's response `ack` propagates back as the manager's response to SysID, so SysID knows it can start its next identification cycle only after MPC actually reloaded.

**FSM states** (all transition guards and thresholds live in `config/adaptive_controller_manager.yaml`):
- `BOOTSTRAP_PP`: startup, PP active, MPC idle. → `RUNNING_PP` once PP confirms `pp_state == true`.
- `RUNNING_PP`: PP in control, SysID identifying online. → `SWITCHING_TO_MPC` once params are identified AND all §Safety-Requirements gates pass.
- `SWITCHING_TO_MPC`: PP deactivated, MPC activated, speed ramp per Design-decision-1 running. → `RUNNING_MPC` once `Δt_switch` elapses and MPC confirms `mpc_health == true`.
- `RUNNING_MPC`: MPC in control, SysID continues re-identification in the background.
- `SWITCHING_TO_PP`: MPC deactivated, PP activated, speed ramp running (same mechanism, reversed roles). → `RUNNING_PP` once `Δt_switch` elapses.
- `EMERGENCY_HALT`: entered from any state if both controllers are unhealthy/timed-out simultaneously (Safety 5A/5B). Publishes hardcoded safe `drive` (zero throttle, zero steering) until at least one controller recovers, then returns to `BOOTSTRAP_PP`.

## 4. `pure_pursuit` (edit existing package)

- New ROS2 param `standalone_mode` (bool, default `false`). `true`: ignore `Start_Working_pp` entirely, behave exactly as the node does today — always publish, no manager required (still runnable/tunable alone via `pure_pursuit.launch.py`, `drive_topic` overridden back to `/drive`). `false` (managed, default): gate publishing behind `Start_Working_pp` as below.
- Subscribe `Start_Working_pp` (`std_msgs/msg/Bool`); when `standalone_mode == false`, gate the existing unconditional `publish()` call in `control_loop()` (`pure_pursuit_node.py:292`) behind this flag — when `Start_Working_pp == false`, skip publishing entirely (do not publish zeros; the manager owns the failsafe).
- Rename `drive_topic` default from `/drive` to `pp/drive_cmd` (`config/pure_pursuit.yaml`, `launch/pure_pursuit.launch.py`).
- Add two new publishers: `pp_state` (`std_msgs/msg/Bool`, true once the control loop has received both odom and waypoints and is actively publishing), `pp_health` (`std_msgs/msg/Bool`, true unless odom/waypoint data is stale beyond a configurable threshold — mirrors the staleness check already implemented in `mpc_path_tracking`'s `applyFallback()`).
- While editing, also fix the pre-existing bug where `launch/pure_pursuit.launch.py`'s declared `DeclareLaunchArgument`s (odom_topic, drive_topic, etc.) are never actually passed into the `Node(...)` block (only `config_file` is) — `mpc_path_tracking`'s launch file already has the correct pattern (`launch/mpc_path_tracking.launch.py:43-50`), copy it.

## 5. `mpc_path_tracking` (edit existing package)

- New ROS2 param `standalone_mode` (bool, default `false`), identical semantics to `pure_pursuit`'s: `true` ignores `Start_Working_mpc`, node behaves as it does today (always publish, no manager needed — useful for solo MPC tuning against the simulator, `drive_topic` overridden back to `/drive`). `false` (managed, default): gate as below.
- Subscribe `Start_Working_mpc` (`std_msgs/msg/Bool`); when `standalone_mode == false`, gate `drive` publishing the same way `pure_pursuit` does.
- Rename `drive_topic` default from `/drive` to `mpc/drive_cmd`.
- Depend on `track_geometry_utils`; replace the inlined projection at `mpc_node.cpp:150-156` with a call to `track_geometry_utils::computeTrackError(...)`.
- Add a service server `mpc/update_params` (`adaptive_controller_interfaces/srv/IdentifiedParam`). On request: validate array size (8), take the tire-params mutex already needed for thread safety, update `TireParams` in place, release, respond `ack = true`. Run this on a **separate callback group** from the control timer so an update never blocks/delays a solve — `ParameterManager`'s existing `onSetParameters` callback (`parameter_manager.hpp`) is the pattern to extend, but this is a service, not a dynamic-parameter callback, since the manager needs a synchronous ack.
- No changes needed to `/mpc/status` — already sufficient as the `mpc_health` source (§Design-decision-3).

## 6. `On-Track-SysID` (edit existing package)

- Publish `sysid/first_run` (`std_msgs/msg/Bool`) once at startup, `true` until the first successful identification, then `false` for the rest of the node's life.
- Replace the one-shot latched `/sysid/training_complete` `String` publish (`on_track_sys_id.py:352-358`) with a service **call** to `sysid/update_params` (client), sending `param_values = [Bf, Cf, Df, Ef, Br, Cr, Dr, Er]` in that fixed order (matches `mpc_path_tracking`'s tire config key order exactly, see grounding note above).
- Block/wait for the service response's `ack` before returning to the "Estimation Phase" (`on_track_sys_id.py:365`, currently unconditional) — only start the next re-identification cycle after `ack == true`, per the existing recursive-loop structure the node already has (`training_complete` phase gate).

# Bringup Launch

One launch file starts the whole autonomous stack together: `adaptive_controller_manager/launch/adaptive_stack.launch.py`. No separate bringup package — the manager is the top-level node in this stack, its launch file is the natural single entry point.

- `IncludeLaunchDescription` for `pure_pursuit/launch/pure_pursuit.launch.py` and `mpc_path_tracking/launch/mpc_path_tracking.launch.py` (both start with `Start_Working_*` gated off internally, i.e. idle-but-alive — the manager, also started here, drives their enable flags per the FSM).
- The `adaptive_controller_manager_node` itself, parameterized from `config/adaptive_controller_manager.yaml`.
- Both includes pass `standalone_mode:=false` explicitly (launch-arg override, not just relying on the YAML default) — a leftover `standalone_mode: true` from a solo-tuning run must never silently survive into a managed launch and defeat the arbiter.
- Shared `DeclareLaunchArgument`s threaded into all three includes: `odom_topic`, `waypoint_topic` (defaults `/odom`, `/raceline_waypoints`, matching both controllers' existing defaults), so renaming either at the top level doesn't require editing multiple files.
- Does **not** itself bring up the simulator or map server — those belong to `f1tenth_simulator/launch/simulator.launch.py`. Real-vehicle bringup includes only `adaptive_stack.launch.py`; simulation bringup includes both (see below), in that order, since the stack's nodes must find `/odom`/`/raceline_waypoints` already latched (`transient_local`) when they start.

## Simulation-only wiring (grounds Offline Simulation Testing below)

`f1tenth_simulator/launch/simulator.launch.py` already runs its **own** `mux` node, which by default also targets `/drive` (`f1tenth_simulator/node/mux.cpp` — the same node cited earlier as prior art for the arbiter pattern) — running it alongside `adaptive_controller_manager` would give `/drive` two independent, uncoordinated publishers. For stack testing, launch the simulator's components directly rather than through `simulator.launch.py` wholesale: `map_server` + `lifecycle_manager` + `racecar_model` include + `simulator` node + `track_publisher.py`, **excluding** `mux`, `behavior_controller`, `random_walker`, and `keyboard` (those exist for manual/random-walk driving, not autonomous-stack testing, and `mux` is the one that would actually conflict). A new test-only launch file, `adaptive_controller_manager/launch/sim_test.launch.py`, includes that trimmed simulator subset plus `adaptive_stack.launch.py`, parameterized by `map_name` (passed straight through to `track_publisher.py`, same mechanism `simulator.launch.py` already uses).

# Offline Simulation Testing

Every behavioral change to the FSM, safety gates, or handover mechanism must be exercised in simulation before considered done — this is a running requirement, not a one-time milestone.

- Use `sim_test.launch.py` (above) against maps in [f1tenth_racetracks](f1tenth_racetracks/) via the existing `map_name` mechanism (`f1tenth_simulator/config/sim.yaml`, default `YasMarina`). Representative subset to cover during development (not exhaustively all 27 — expand only if a specific track exposes a bug): `YasMarina` (existing default, moderate corners), `Sochi` (tight low-speed corners — stresses the `v_min`/alignment arming gates), `Monza` (long straights — stresses the bootstrap→arm transition at high approach speed and the speed-ramp handover).
- Minimum acceptance criteria per track, per change:
  - Full FSM cycle observed at least once: `BOOTSTRAP_PP → RUNNING_PP → SWITCHING_TO_MPC → RUNNING_MPC`, and back via `SWITCHING_TO_PP` when forced (e.g. by killing/restarting the MPC node mid-lap to trigger the health-check fallback).
  - No double-publish on `/drive` — confirm via `ros2 topic info /drive --verbose` showing exactly one publisher (the manager) while the stack is up.
  - `pp/drive_cmd`/`mpc/drive_cmd` each only carry non-idle commands while their `Start_Working_*` flag is true (idle-but-alive nodes should not emit steering/speed noise while gated off).
  - Handover produces no steering discontinuity and a monotonic speed ramp over `Δt_switch` — check by echoing `/drive` around the switch instant.
  - Injected MPC failure (e.g. temporarily corrupt `mpc/update_params` values past the manager's plausibility bounds, or kill the MPC process) reliably drives `EMERGENCY_HALT` or `SWITCHING_TO_PP` within `Δt_timeout`, never a silent stall.

# Custom Interfaces

## `adaptive_controller_interfaces/srv/IdentifiedParam.srv`

```
# Tire Pacejka params, fixed order: [Bf, Cf, Df, Ef, Br, Cr, Dr, Er]
# matches mpc_path_tracking/config/mpc_path_tracking.yaml's tire: block
float32[8] param_values
---
bool ack
```

Used for both hops (SysID→manager, manager→MPC) — same type, two independent service instances (`sysid/update_params`, `mpc/update_params`), each with its own server/client pairing as listed above.

# Step-by-Step Workflow

## 1. Bootstrap Phase

- `On-Track-SysID` publishes `sysid/first_run = true` at startup.
- `adaptive_controller_manager` starts in `BOOTSTRAP_PP`: publishes `Start_Working_pp = true`, `Start_Working_mpc = false`.
- `pure_pursuit` starts publishing `pp/drive_cmd`; the manager forwards it to `/drive` unmodified (arbiter passthrough, single active source).
- `On-Track-SysID` observes vehicle behavior via `/odom` and `/drive` (the manager's arbitrated output — SysID reads the actual command sent to the vehicle regardless of source, so its subscription target does **not** change from today's `/drive`).

## 2. Parameter Estimation

- SysID computes Pacejka `[B,C,D,E]` per axle as today (`on_track_sys_id.py`'s existing NN/`solve_pacejka` pipeline — unchanged).
- On convergence, SysID calls `sysid/update_params` with the 8-value array and blocks for the response.
- The manager's service server validates bounds (config-driven plausible ranges), stores the params, and does **not** yet forward them to MPC — forwarding is gated on the Safety Requirements below being satisfied first. If not yet satisfied, the manager still responds `ack = true` to SysID (params accepted/stored) so SysID can keep refining, but internally queues the latest accepted params for the next arming attempt.

## 3. Controller Handover (`RUNNING_PP` → `SWITCHING_TO_MPC` → `RUNNING_MPC`)

- Once all Safety Requirement gates pass (below):
  - Manager calls `mpc/update_params` (client, own callback group, non-blocking w.r.t. the arbitration timer) with the stored params.
  - On `ack == true` from MPC, manager also acks SysID's original request (if still pending) and transitions to `SWITCHING_TO_MPC`.
  - Publishes `Start_Working_mpc = true`, `Start_Working_pp = false`.
  - Runs the speed ramp per §Design-decision-1 for `Δt_switch`, forwarding `mpc/drive_cmd`'s steering unmodified and blending its speed.
  - Transitions to `RUNNING_MPC` once `Δt_switch` elapses and `mpc_health == true` throughout.
- SysID continues re-identification in the background (`RUNNING_MPC` state); each subsequent convergence repeats step 2's `sysid/update_params` call, and the manager forwards to `mpc/update_params` immediately (no re-arming safety gates needed for in-place re-identification, only for the initial PP→MPC handover) — this is the recursive loop described in the original draft's §2 Controller Manager parameter update loop.

# Safety Requirements for Switching

(Kept from the original draft, updated with concrete sources.)

## 1. Vehicle State Validation

- **A. Speed threshold:** `v_x > v_min` (config, e.g. 0.5 m/s), sourced from `/odom` twist. Below threshold, remain in Pure Pursuit.
- **B. Track alignment:** `|e_y| < e_y_max` (e.g. 0.1 m) and `|Δθ| < θ_max` (e.g. 0.1 rad), sourced from `track_geometry_utils::computeTrackError` computed independently by the manager (§Design-decision-2) — available continuously, not gated on MPC's debug flag.

## 2. System Health Checks

- **A. MPC solver health:** `mpc_health := (/mpc/status.level == DiagnosticStatus::OK)` (§Design-decision-3). If it drops to `false` while `RUNNING_MPC`, immediately transition to `SWITCHING_TO_PP`.
- **B. Data freshness:** `Δt` since last `/odom` message `< Δt_state_max` (e.g. 0.1 s). Stale data blocks switching; if MPC is active and data goes stale, fall back to Pure Pursuit.

## 3. Control Performance Validation

- **Error convergence:** running average of `d(e_y)/dt` over the last N samples (config) must be `< 0` (or within a small positive tolerance) before arming MPC — don't hand over into an already-diverging error.

## 4. Smooth Handover

- Implemented exactly per §Design-decision-1 (instant steering, ramped speed via frozen start value + live target, `Δt_switch` linear ramp). No `predicted_initial_state` topic is required for this mechanism — steering is taken directly from the newly active controller, so there is no "initial state prediction" step to validate; this simplifies the original draft's §4A, which is dropped as unnecessary under the resolved mechanism.

## 5. Emergency Failsafes

- **A. Controller timeout:** `Δt_timeout = 1.0 s` — if the active controller's `*_state` stops updating (PP) or `/mpc/status` stops arriving (MPC) for this long, attempt switch to the other controller if healthy.
- **B. Total failure → `EMERGENCY_HALT`:** if both controllers are unhealthy/timed-out simultaneously, publish a hardcoded zero-throttle, zero-steering `drive` command (mirrors `f1tenth_simulator/node/mux.cpp`'s "no active channel → zero command" failsafe) until at least one controller recovers.

# Configuration Files

## `mpc_path_tracking/config/mpc_path_tracking.yaml` (existing, add)

- `topics.enable_topic: "Start_Working_mpc"`, `topics.drive_topic: "mpc/drive_cmd"` (rename), `topics.param_service: "mpc/update_params"`, `standalone_mode: false`.

## `pure_pursuit/config/pure_pursuit.yaml` (existing, add)

- `enable_topic: "Start_Working_pp"`, `drive_topic: "pp/drive_cmd"` (rename), `state_topic: "pp_state"`, `health_topic: "pp_health"`, `stale_data_timeout_s` (new, for `pp_health`), `standalone_mode: false`.

## `adaptive_controller_manager/config/adaptive_controller_manager.yaml` (new)

- `v_min`, `e_y_max`, `theta_max`, `delta_t_state_max`, `error_convergence_window`, `delta_t_timeout`, `delta_t_switch`, tire-param plausible bounds (min/max per one of the 8 values, for the manager's validation step), topic name overrides (all topics above), diagnostics/logging verbosity.

# Implementation Notes

- ROS2 distro: build/test on the host's installed **Humble** (`/opt/ros/humble`); the Docker image targets **Jazzy** (`docker/dockerfile`) — both must keep working, don't assume one implies the other.
- New C++ packages follow `mpc_path_tracking`'s structure (separate `include/<pkg>/*.hpp` + `src/*.cpp`, `ParameterManager`-style param declaration, `ament_cmake`) rather than `joy_to_ackermann`'s flatter layout, for consistency with the most recently-built package in this stack.
- Any new `package.xml` maintainer email must use a dotted domain (e.g. `ebrahim@example.com`) — a bare `ebrahim@local` fails `catkin_pkg` validation and aborts `colcon build` (hit this before with `estimation_benchmark`).
- Standardize on `colcon build --symlink-install` for every new/edited package in this stack — don't mix build modes on the same package across sessions (`rm -rf build/<pkg> install/<pkg>` first if switching mode is ever unavoidable).
- Service callbacks that mutate live controller state (MPC's `mpc/update_params` server) must run on a callback group separate from the control-loop timer, and must take the shortest possible lock around the mutation — the control loop must never block on a service call.
