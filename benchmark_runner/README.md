# benchmark_runner

Runs a sweep of identification + control scenarios end to end and builds a
cross-scenario comparison from the results. Replaces the two-terminal,
edit-the-makefile workflow that produced `graphs/identification/*` and
`graphs/control/*` by hand.

## Prerequisite: start the simulator yourself

```bash
cd /home/ebrahim/Carla_ASU_Bridge
make launch_carla_sim AUTO_START=false
```

Leave that running. The runner drives the bridge's **lifecycle**
(`configure`/`activate` before each scenario, `deactivate`/`cleanup` after) but
never starts or kills the CARLA server or the bridge process. `AUTO_START=false`
matters: the runner does the configure/activate itself, per scenario. If the
lifecycle services are missing it says so and stops.

## Run

```bash
make run_benchmark_scenarios                       # every scenario in scenarios.yaml
make run_benchmark_scenarios ONLY=test1_with_base_line
make run_benchmark_scenarios SCENARIOS=/path/to/other.yaml
make compare_benchmark_scenarios                   # cross-scenario figures
```

## What one scenario does

1. Wipe `graphs/identification/<name>/` and `graphs/control/<name>/` — overwrite
   means overwrite, no figure or CSV from a previous run survives.
2. `configure` → `activate` the bridge; wait for `/odom` and
   `/sim/feedback/tire_forces`.
3. Launch `raceline_publisher`; wait for `/raceline_waypoints`.
4. Launch `on_track_sys_id` + `tire_force_benchmark`, with
   `SYSID_NN_PARAMS_FILE` / `SYSID_PACEJKA_PARAMS_FILE` pointing at this
   scenario's yaml pair.
5. Launch the adaptive control stack + `adaptive_controller_benchmark`. The
   manager is the sole `/drive` writer, so the car starts moving here.
6. Start the friction schedule for this scenario.
7. Drive `n_laps` **timed** laps.
8. SIGINT identification, wait for it to finish exporting; then the control
   stack; then the raceline publisher. Both benchmark nodes write their figures
   from `destroy_node()`, so the shutdown has to be graceful and unhurried —
   `tire_force_benchmark` ignores signals while exporting and takes ~10-15 s.
9. `deactivate` → `cleanup` the bridge, then verify with `pgrep` that nothing
   survived. A survivor aborts the sweep rather than contaminating the next
   scenario.

### Laps and the out-lap

The car spawns **behind** the raceline's `s = 0`, so the drive to the start line
is an out-lap and is not counted. `n_laps: 6` means 6 timed laps *plus* that
out-lap, i.e. 7 `s_m` wraparounds, and `lap_times.csv` comes out with 6 rows.
The runner reuses `adaptive_controller_benchmark`'s own `LapTracker`, so its
count and the count in `lap_times.png` cannot disagree.

## scenarios.yaml

| Key | Meaning |
|---|---|
| `n_laps` | Timed laps per scenario |
| `lap_timeout_s` | Abort a scenario that stops making progress |
| `raceline_csv`, `psi_offset_rad` | Passed to `raceline_publisher`. Use `1.5707963268` for `traj_race_cl.csv` (raw TUM output), `0.0` for `f1tenth_racetracks/*` |
| `graphs_root` | Output root, default `graphs` |
| `nominal_tire_friction` | The configured per-wheel coefficient the schedules start from |
| `scenarios[].nn_params_yaml` / `pacejka_params_yaml` | This scenario's parameter set |
| `scenarios[].friction_schedule` | See below |

### Friction schedules

Driven over `/sim/control/tire_friction` (`std_msgs/Float32`) at the simulator's
own 0.033 s step, the mechanism `paper/sections/experiments.tex` §VI-I
specifies. The commanded value is the **configured** coefficient; PhysX
multiplies it by the road-surface factor (0.70 on silverstone), and that
effective value comes back on `/sim/feedback/tire_forces.tire_friction`, which
is what the benchmark scores against. `raw/mu_commanded.csv` records the
command.

| Value | Behaviour |
|---|---|
| `constant` | Hold `nominal_tire_friction` |
| `decay_2pct_s` | Linear decay of 2 %/s (LLA-MPC Experiment 1) |
| `step40_end_lap1` | 40 % drop when the first **timed** lap completes (Experiment 2) |
| `step40_mid_lap1` | 40 % drop halfway through the first timed lap, by arc length (Experiment 3) |

**The three varying-friction schedules are blocked** on regenerating the
raceline (`paper/sections/experiments.tex` §VI-J): `traj_race_cl.csv` demands
1.22 g against a plant that sustains ~1.0 g, so at 60 % of nominal grip the car
is being asked to drive a trajectory that does not exist. They run, and what
happens is recorded, but a clean Experiment 2/3 result needs the regenerated
raceline first. They are commented out in `scenarios.yaml` for that reason.

## Outputs

```
graphs/identification/<scenario>/   tire_force_benchmark PNGs + one CSV per PNG
                        raw/        tire_forces.csv, tire_forces_states.csv,
                                    mu_commanded.csv, on_track_sys_id.log
graphs/control/<scenario>/          adaptive_controller_benchmark PNGs + CSVs
                        raw/        controller.csv, *.log
graphs/comparison/                  cross-scenario figures + comparison_summary.csv
```

Every exported PNG has a same-named CSV holding the numbers behind it.
`compare_scenarios.py` reads those rather than re-deriving any metric or tire
curve, so the comparison and the per-scenario figures cannot disagree.

`comparison_summary.csv` has one row per scenario: laps, best/mean lap time, RMS
and Max `|e_y|`, ITAE, heading error, transitions, emergency halts, MPC solve
time, per-axle tire-force RMSE and R², μ error, `v_y`/`ω` RMSE and the
identified `[B, C, D, E]` per axle.

## Tests

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m pytest benchmark_runner/test -q
```

The lap tests deliberately start the synthetic car partway round the circuit,
because that is what the simulator does.
