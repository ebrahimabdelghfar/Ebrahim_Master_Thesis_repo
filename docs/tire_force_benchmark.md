# tire_force_benchmark

Benchmarks a nominal Pacejka tire model — identified by `On-Track-SysID` (indirect, NN-assisted, on-track) or `pacejka_identification` (direct, offline, from simulated ground truth) — against CARLA's simulated per-wheel tire forces, and separately benchmarks a one-step vehicle-state (v_y, ω) prediction built from that same nominal model against odometry. See package README (`tire_force_benchmark/README.md`) for parameters, topics, and run instructions; this document covers the underlying model, what's actually being validated, and the bugs found/fixed while reviewing the implementation.

## Notation

| Symbol | Description | Unit |
|--------|-------------|------|
| `α` (`alpha`) | Tire slip angle | rad |
| `Fz` | Normal (vertical) tire load | N |
| `Fy` | Lateral tire force | N |
| `B` | Pacejka stiffness factor — slope at zero slip (cornering stiffness `= B·C·D`) | — |
| `C` | Pacejka shape factor | — |
| `D` | Pacejka peak factor (peak force `= D·Fz`) | — |
| `E` | Pacejka curvature factor | — |
| `v_x`, `v_y` | Longitudinal / lateral velocity, body frame | m/s |
| `ω` (`omega`) | Yaw rate | rad/s |
| `δ` (`delta`) | Front steering angle | rad |
| `m` | Vehicle mass | kg |
| `I_z` | Yaw moment of inertia | kg·m² |
| `l_f`, `l_r`, `l_wb` | CoG-to-front-axle, CoG-to-rear-axle, wheelbase (`l_f+l_r`) | m |

## What "ground truth" actually is

`/sim/feedback/tire_forces` (`sim_manager_msgs/TireForces`) is CARLA's own per-wheel telemetry (`Vehicle::GetTelemetryData`), i.e. PhysX's contact-patch state, **not** measured from physical sensors. Of its fields only `lateral_force` is usable as force ground truth: it tracks `m·a_y` in steady cornering (corr +0.95 to +0.97), while `longitudinal_force` behaves as a friction-capacity report (pinned at `tire_friction·normal_load`, corr +0.04 with `m·a_x`) — see the field-by-field caveats in `sim_manager_msgs/msg/TireForces.msg`. `lateral_force` is published sign-flipped into the ROS body frame so it shares its sign convention with `slip_angle`, matching this repo's Pacejka convention (`Fy = pacejka(α)` with `α = δ - atan((v_y + l_f·ω)/v_x)` at the front).

Benchmarking a model that was itself fit from the same simulator (`On-Track-SysID/models/SIM/SIM_pacejka.txt`, or a `pacejka_identification` run on this same topic) is closer to an in-sample self-check than a sim-to-real validation. A genuine "nominal model learned on real hardware, validated in simulation" experiment requires benchmarking one of the physical-car models (`NUC2`/`NUC5`/`NUC6`/`NUC7`, all fit from real on-track data via `On-Track-SysID`) against the CARLA feed instead. The odometry/steering path used for vehicle-state benchmarking (`/odom`, `/drive`) is agnostic to this distinction — it will benchmark against whatever those topics actually carry (real state estimator or simulated).

Two sample gates follow from the message's own documented behaviour: `|normal_load| >= min_fz_threshold` per wheel, and a standstill drop — below 0.5 m/s the publisher zeroes `slip_angle` and both forces while `normal_load` stays live, so an all-zero slip/`lateral_force` frame is rejected rather than scored as a perfect (0, 0) sample.

## Pacejka Magic Formula (as implemented)

$$
F_y(\alpha) = F_z \cdot D \cdot \sin\!\Big(C \cdot \arctan\big(B\alpha - E(B\alpha - \arctan(B\alpha))\big)\Big)
$$

Implemented identically (and verified byte-for-byte consistent) in `tire_force_benchmark_node.py`'s `pacejka_formula()`, `On-Track-SysID/src/helpers/pacejka_formula.py`, and `pacejka_identification`. Coefficient order throughout this codebase is `[B, C, D, E]`.

On shutdown (`plot_output_dir` set), `pacejka_curve_validation.png` renders this curve directly — measured `(α, Fy)` scatter per axle against the model swept over the observed slip-angle range at the nominal static axle load, mirroring the field's standard validation figure (Bakker/Nyborg/Pacejka SAE 870421; Pacejka & Bakker 1992; Fig. 6-style plots in Dikici et al. 2024). The parity plots (`*_parity.png`) provide the complementary estimate-vs-ground-truth regression view used for the RMSE-based validation in the same references. See `tire_force_benchmark/README.md`'s "Academic plot export" section for the full plot list.

## One-step vehicle-state prediction (as implemented)

Front/rear slip angles from the standard bicycle-model kinematics:

$$
\alpha_f = -\arctan\!\left(\frac{v_y + \omega l_f}{v_x}\right) + \delta,
\qquad
\alpha_r = -\arctan\!\left(\frac{v_y - \omega l_r}{v_x}\right)
$$

static axle loads `Fzf = m g l_r / l_wb`, `Fzr = m g l_f / l_wb`, Pacejka forces `F_f = F_y(α_f; Fzf)`, `F_r = F_y(α_r; Fzr)`, then the lateral/yaw rigid-body force-moment balance, Euler-integrated one step:

$$
\dot v_y = \frac{1}{m}\big(F_r + F_f\cos\delta - m v_x \omega\big),
\qquad
\dot\omega = \frac{1}{I_z}\big(F_f l_f \cos\delta - F_r l_r\big)
$$

$$
v_y[t+1] = v_y[t] + \dot v_y\, \Delta t,
\qquad
\omega[t+1] = \omega[t] + \dot\omega\, \Delta t
$$

This is the same math already used (and presumably validated in the source paper, Dikici et al. 2024) by `On-Track-SysID/src/on_track_sys_id.py`'s `publish_estimates()` / `generate_predictions.py` — `tire_force_benchmark` re-implements it independently (rather than subscribing to that node's output) so it can benchmark the nominal model's state-prediction accuracy standalone, using the real inter-sample `dt` from odometry timestamps and the same guards against a non-monotonic timestamp, an untrustworthy timing gap (`dt > 0.2s`), or a stopped vehicle (`v_x < 0.1 m/s`, since the slip-angle formula divides by `v_x`).

## Identified-params service (no hardcoded model)

Originally `c_pf`/`c_pr` had hardcoded fallback defaults, meaning the node could silently benchmark an arbitrary Pacejka model nobody actually identified. This was replaced with:

- No built-in default for `c_pf`/`c_pr` — they come from `model_file`, explicit parameters, or a new `identified_params_service` (`adaptive_controller_interfaces/srv/IdentifiedParam`, the same request/response contract already used for the `On-Track-SysID → adaptive_controller_manager → mpc_path_tracking` handoff, so it's a natural extension rather than a new interface).
- `internal_pacejka` Fy benchmarking and vehicle-state prediction are both held off (`have_identified_params == False`) until one of those three sources provides a model — a wrong/never-identified model isn't a meaningful thing to benchmark against.
- `adaptive_controller_manager` optionally forwards (`benchmark_update_params_enable`, default `false`) every accepted `sysid/update_params` submission on to this service, fire-and-forget, directly from `onSysidUpdateParams()` — deliberately **not** routed through the same arming-FSM gate as the `mpc/update_params` forward, since this consumer is a passive observer, not an actuator; it should see every accepted identification immediately. See `docs/adaptive_controller_manager.md`.

## Bugs found and fixed

### 1. External-mode queue-alignment drift

`_try_external_queue_alignment()` is supposed to implement "compare `estimate[k]` with `ground_truth[k+lead]`" (the package's own documented contract). The previous implementation popped `lead + 1` ground-truth samples off the queue for every one estimate consumed, instead of sliding the ground-truth window forward by exactly one sample per estimate. Traced by hand for `lead=1`: pair 0 `(e0, g1)` is correct by chance, but pair 1 becomes `(e1, g3)` instead of `(e1, g2)`, pair 2 becomes `(e2, g5)` instead of `(e2, g3)` — the misalignment grows by one extra ground-truth sample every iteration. Only `lead=0` was ever correct. This silently corrupted every metric in `external_topic` mode whenever a lead ≥ 1 was configured (the package's own recommended one-step-ahead setting).

**Fix:** pop exactly one ground-truth sample per estimate consumed. Covered by `test/test_queue_alignment.py::test_queue_alignment_lead_two` — confirmed to fail against the pre-fix logic (reproducing the exact drift pattern above) and pass after the fix.

### 2. `min_fz_threshold` default was physically impossible for this repo's vehicles

The previous default (50 N) exceeds the *entire* static weight (`m·g`) of every vehicle model tracked in this repo (`On-Track-SysID/models/*`, all 3.5–4.5 kg scaled cars, static per-wheel load ≈ 8.6–11 N) — every sample would have been silently rejected, with no error, producing an empty benchmark. There is now no built-in default; the parameter must be set explicitly (`config/benchmark_config.yaml` or `-p min_fz_threshold:=<value>`), sized well below `mass·9.81/4` N so it only filters near-zero/airborne-wheel samples. Covered by `test_min_fz_threshold_missing_raises` / `test_min_fz_threshold_present_constructs_and_filters`.

### 3. Package was never actually installed by colcon

`tire_force_benchmark/tire_force_benchmark/` (the Python module directory) had no `__init__.py`. `setuptools.find_packages()` requires one to recognize a directory as a regular package, so the `tire_force_benchmark` subpackage was silently excluded from `packages=find_packages(...)` in `setup.py` — confirmed by inspecting a prior `colcon build` output, which installed the launch file but no `.py` module at all. `ros2 run tire_force_benchmark tire_force_benchmark_node` would have failed with `ModuleNotFoundError`. **Fix:** added the missing `__init__.py`; a fresh `colcon build --packages-select tire_force_benchmark` now installs and imports the module correctly (verified).

### 4. `On-Track-SysID`: `num_of_iterations` config key was dead

`On-Track-SysID/README.md` documents `params/nn_params.yaml: num_of_iterations` as user-adjustable ("If the model is not converging after 6th iteration, it can be increased"), but `src/helpers/train_model.py`'s `nn_train()` hardcoded `num_of_iterations = 6` and the key never existed in `nn_params.yaml` — editing it per the README had zero effect. **Fix:** `num_of_iterations = nn_params.get('num_of_iterations', 6)`, key added to `nn_params.yaml`. Verified with a stubbed-training-function script (heavy NN/Pacejka/LUT calls replaced with call-counters) confirming the loop now runs the configured number of times instead of always 6.

## References

- Pacejka, H.B., *Tire and Vehicle Dynamics*, 3rd Ed., Ch. 4 — Magic Formula tire model.
- Bakker, E., Nyborg, L., Pacejka, H.B., *"Tyre Modelling for Use in Vehicle Dynamics Studies"*, SAE Technical Paper 870421, 1987 — original Magic Formula formulation.
- Pacejka, H.B. and Bakker, E., *"The Magic Formula Tyre Model"*, Vehicle System Dynamics, 21:sup1, 1-18, 1992.
- Dikici, O., Ghignone, E., Hu, C., Baumann, N., Xie, L., Carron, A., Magno, M., Corno, M., *"Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute"*, arXiv:2411.17508, 2024 — the on-track NN+Pacejka identification method whose nominal model this package benchmarks.
