# Figure and table production guide

Each entry gives: the data source, the plot type, the axes, and a generation
recipe. Export PDF for vector plots, PNG only for screenshots.

Figures 1–4 are declared in `sections/platform.tex` and `sections/method.tex`
so they float beside their text; the rest are in `sections/experiments.tex` and
come from `graphs/comparison/` (see the results-figures section below).
Numbering follows declaration order, so read the authoritative numbers out of
`main.aux` (`grep newlabel`) rather than counting headings here.

Common data capture:

```bash
# 30 s identification run, everything the figures below need
ros2 bag record -o sysid_run \
  /odom /drive \
  /sim/feedback/steering_angle /sim/feedback/steering_angles \
  /sim/feedback/imu /sim/feedback/tire_forces /sim/feedback/speed \
  /raceline_waypoints /clock
```

`On-Track-SysID` also exports the collected buffer directly to CSV
(`export_data_as_csv`), which is the simpler source for figures 4, 6, 7, 8, 9.

---

## Fig. 3 — `frames.tex` — Frame and sign conventions  ✅ DONE

**Type:** three-panel convention diagram. Vector TikZ, `\input` by
`sections/platform.tex` inside a `figure*` at `\textwidth`.

(a) CARLA/Unreal's left-handed world (X east, Y south, Z up, yaw CW) against
ROS 2's right-handed ENU (REP-103), with the map between them. (b) The vehicle
body frame every identification quantity is written in: FLU, `ω`/`δ` positive
counter-clockwise, `α` and `Fy` positive to the left. (c) The per-channel
conversion `Carla_ASU_Bridge` applies — including the one channel
(`tire_forces` slip angle) that needs no sign change, because the frame flip
and CARLA's own convention cancel.

**If it is edited:** panel boxes are fitted to explicit corner coordinates
(`pa1/pa2`, `pb1/pb2`, `pc1/pc2`), not to their content, so the three panels
keep the same height under `\resizebox`. `fit=` cannot take a raw `(-4mm,-6mm)`
coordinate — TikZ parses the minus sign and fails; name the coordinate first.

**Note:** this figure took number 3, so every later figure shifted by one
relative to the headings below. The authoritative numbering is
`grep newlabel ../main.aux`, not these headings.

---

## Fig. 4 — `pipeline.tex` — Identification pipeline  ✅ DONE

**Type:** block diagram with a feedback loop. Vector TikZ, `\input` by
`sections/method.tex` inside a `figure*` at `0.98\textwidth`.

Stages left to right: recorded lap → residual NN training → synthetic steering
sweep → steady-state force recovery and bounded Pacejka fit → re-nomination,
with the loop closed along the bottom of the diagram and an exit downward
through the admissibility gate to the MPC. Green marks this work's changes
(friction warm-start floor, rollout speed selection, sweep bound
`1.5·P99(|δ|)`, frozen regularisation target); blue is the gated model
interface; orange is the plant's force telemetry, drawn as scoring-only so it
is visibly outside the loop.

**If it is edited:** keep the bottom loop-back routing. Any route through the
middle of the diagram runs behind the gate and telemetry boxes.

---

## Fig. 5 — `pipeline_s4.tex` — S4D residual and augmented inputs  ✅ DONE

**Type:** three-panel block diagram. Vector TikZ, `\input` by
`sections/method.tex` inside a `figure*` at `0.98\textwidth`.

Panel (a) input construction: the four measured signals, the two slip angles
appended by `preprocess_inputs()`, the `W = 20` windowing from
`generate_sequence_windows()` (no window may cross the seam between the trace
and its mirrored copy), and the resulting `(N, W, 6)` tensor. Panel (b) the
stack of `helpers/s4_residual.py`: input projection to `H = 4`, S4D blocks with
diagonal `A` and S4D-Lin init, residual + LeakyReLU, output projection to
`[e_vy, e_omega]`. Panel (c) the two evaluation paths of the same LTI system —
`forward()`/`forward_last()` for training, `step()` for `simulated_data_gen()`'s
500-step rollout — with the device each runs on and the measured cost.

**Source of the numbers in panel (c):** Table IX; they are the profiling
figures recorded in `.wolf/memory.md` for 2026-09-01, not estimates. Re-measure
with the scratchpad harness if the architecture config changes.

---

## Fig. 1 — `fig2.png` — Formula AI vehicle in CARLA  ✅ DONE

**Type:** screenshot. Front view of `vehicle.vehicle.asurt_fsai` on the
start/finish straight of the `silverstone` map. 524×457 px RGBA, included at
`0.86\columnwidth`, floats on page 5 next to the Method platform subsection.

**Optional improvement:** pair it with a top-view schematic panel annotated
with `l_f = 0.738 m`, `l_r = 0.795 m`, `L = 1.533 m`, wheel radius 0.25 m,
±16° road-wheel limit and the IMU/GNSS mount points (both at the vehicle
origin), so the geometry is readable off the figure rather than only from the
caption. Draw it in the same tool as Fig. 1/Fig. 3 and set the two panels side
by side with `subfigure` or a two-column `tabular`.

**If the screenshot is ever re-taken:** enable `third_person_view` in
`Carla_ASU_Bridge/config/carla_interface_config.yaml`, then

```bash
ros2 run image_view image_saver \
  --ros-args -r image:=/third_person_view/raw_images \
  -p filename_format:=fsai_%04d.png
```

---

## Fig. 2 — `bridge_architecture.tex` — Carla_ASU_Bridge architecture  ✅ DONE

**Type:** vector TikZ, authored directly in LaTeX (no external tool, no PDF to
regenerate). Lives in `figures/bridge_architecture.tex`, pulled into a
`figure*` in `sections/method.tex` via
`\resizebox{\textwidth}{!}{\input{...}}`.

**Layout:** four columns, left to right — CARLA server (PhysX ego, sensor
actors, world tick, wheel telemetry) → `Carla_ASU_Bridge` lifecycle node
(control application, per-sensor threads, telemetry thread, `sensor_clock`) →
ROS 2 topic layer (subscribed / published / ground truth) → autonomy stack
(`pure_pursuit`, `On-Track-SysID`, `mpc_path_tracking`,
`adaptive_controller_manager`, benchmark nodes).

**Two highlighted paths, both load-bearing for the paper's argument:**

- **Orange** — `feedback/tire_forces`, the ground truth. It is drawn reaching
  *only* the benchmark nodes, labelled "validation only", because it must be
  visible that this signal scores the identifier and is never wired into the
  identification loop. Getting this wrong in the figure would undercut the
  paper's central validity claim.
- **Blue** — `mpc/update_params`, the gated model interface, drawn *inside*
  the autonomy stack rather than in the topic column: it is a service between
  two of this workspace's own nodes, not part of the bridge's topic layer.

**Editing notes:**

- Boxes stack with `below=<gap> of <prev>`, never absolute `y`, so rewording a
  box cannot make two overlap. Do **not** add `anchor=north west` to the
  shared `blk` style — it fights `positioning` and cascades the column
  diagonally. Only the first node of each column carries an explicit anchor.
- **Every inter-column arrow begins and ends on a dashed group boundary**
  (`topics.west`, `stack.west`, …), never on an inner box. An arrow that
  pierces the dashed frame reads as though it bypassed the column. The row an
  arrow belongs to is carried by the `y` coordinate, e.g.
  `(bridge.east |- tgt) -- (topics.west |- tgt)`, so it still lines up with
  its own topic.
- Horizontal arrows between two groups must use the `|-` / `-|` intersection
  syntax. The four group frames have **different heights**, so `carla.east`
  and `bridge.west` do not share a `y` and a plain `(carla.east) --
  (bridge.west)` comes out slanted.
- Column `x` positions are the four `at (x,0)` coordinates. The gutter between
  columns 1 and 2 must stay wide enough for the two-line `libcarla_client`
  arrow label.
- The whole picture is ~19.5 cm wide and resized to `\textwidth`, i.e. very
  close to 1:1 — keep it near that width or the type will shrink.

---

## Fig. 6 — `raceline.pdf` — The optimised raceline  ✅ DONE

**Generated by:** `python3 figures/make_raceline_figure.py` (re-run it after any
change to `traj_race_cl.csv`; it prints the numbers quoted in the text so they
can be re-checked). Vector PDF, sized for a `figure*`.

**Source data:** `traj_race_cl.csv` at the repo root — byte-identical
(`md5 8e99f36…`) to `outputs/traj_race_cl.csv` inside
`Carla_ASU_Bridge/src/ros_apps/global_racetrajectory_optimization`, i.e. the
optimiser's own output, not a copy that drifted.

**Provenance, all verifiable from that package:**

| Item | Value | Where |
|---|---|---|
| Input centreline | `inputs/tracks/handling_track_recorded.csv`, 15465 pts | span 1018.7 × 1721.5 m vs output 1014.7 × 1720.8 m |
| Formulation | minimum curvature (`optim_opts_mincurv`) | `params/racecar.ini` |
| Assumed mass | **204 kg** (plant measures 269.6 kg) | `params/racecar.ini` |
| Assumed grip | **flat `ay_max = 12.0 m/s²` at every speed** | `inputs/veh_dyn_info/ggv.csv` |
| `v_max`, `width_opt`, `curvlim` | 60 m/s, 3.4 m, 0.0981 1/m | `params/racecar.ini` |
| Output | 2916 pts @ 2.0 m, 5830.2 m lap, 14.90–46.02 m/s | `traj_race_cl.csv` |

**Panels:** (a) track layout coloured by `vx`; (b) speed profile vs `s`;
(c) lateral demand `vx²|κ|` vs `s` against the assumed and the measured grip.

**Why panel (c) carries the paper's negative result:** peak demand is
`12.00 m/s²` — the assumed envelope, exactly — against a measured ceiling of
`≈9.8 m/s²`. 11.9 % of the lap is infeasible. The raceline and the
identification were parameterised from two different descriptions of the same
car.

**Design constraints honoured** (dataviz skill — keep these on any redraw):

- **No dual axes.** Speed and demand are separate panels. Twinning them would
  be the single most common chart error and would also imply a relationship
  between the two scales that does not exist.
- Speed is a *magnitude*, so it gets **one hue, light→dark** (blue ramp steps
  200→700), never a rainbow. Started at step 200 so the slow end stays visible
  as a thin line on white paper.
- Single-series panels carry **no legend box**; the two reference levels are
  direct-labelled, so meaning is never colour-alone.
- Colours are the validated defaults: series-1 `#2a78d6`, status-critical
  `#d03b3b`. The pair was checked with the skill's validator
  (`node scripts/validate_palette.js "#2a78d6,#d03b3b" --mode light`) — all six
  checks pass, CVD ΔE 23.8, normal-vision ΔE 31.6.
- The "measured ceiling" label sits in the band above 12.0 m/s², which is
  empty by construction, with a leader down to its line. Anchoring it to the
  trace cannot work — no stretch of this lap is both calm enough and wide
  enough — and its leader `x` is picked from the data by `calmest_point()`,
  not by eye, so it survives a change of track.

---
## Results figures — `../../graphs/comparison/`

Every results figure in `sections/experiments.tex` is produced by the benchmark
sweep, not by a script in this directory, and is included straight from
`graphs/comparison/` (see `main.tex`'s `\graphicspath`). Regenerate with:

```bash
# simulator started by hand first: cd ~/Carla_ASU_Bridge && make launch_carla_sim AUTO_START=false
make run_benchmark_scenarios      # one run per scenario in benchmark_runner/scenarios.yaml
make compare_benchmark_scenarios  # graphs/comparison/*.png + one CSV per PNG
```

The paper uses all of them:

| Figure in the paper | File |
|---|---|
| Identified vs. nominal tire model | `pacejka_identified_vs_nominal_by_scenario.png` |
| Peak-friction error per axle | `mu_error_by_scenario.png` |
| Peak friction over the run | `mu_timeseries_by_scenario.png` |
| Axle force RMSE / R² | `tire_force_rmse_by_scenario.png`, `tire_force_r2_by_scenario.png` |
| Axle force error distribution | `tire_force_error_hist_by_scenario.png` |
| Axle force against truth | `tire_force_timeseries_by_scenario.png` |
| One-step state RMSE vs. persistence | `state_rmse_by_scenario.png` |
| State error distributions | `state_error_hist_v_y.png`, `state_error_hist_omega.png` |
| States against truth | `state_timeseries_by_scenario.png` |
| Tracking error by controller | `tracking_error_by_scenario.png` |
| Signed tracking error | `e_y_error_hist_by_scenario.png` |
| ITAE by controller | `itae_by_scenario.png` |

Each PNG ships a same-named CSV holding the samples behind it, and
`comparison_summary.csv` holds the one row per scenario that Table V reports.
Numbers quoted in the text come from those CSVs; nothing is re-derived here.

`lap_times_by_scenario` and `mpc_solve_time_by_scenario` are written by
`compare_scenarios.py` but are not used in the paper: lap timing runs on the
ROS wall clock against a faster-than-real-time simulator, and the solve-time
figure carries two numbers that fit in Table V.

---

## Tables

| Table | Content | Source |
|---|---|---|
| I | Vehicle configured vs measured | `carla_interface_config.yaml`; `feedback/tire_forces` at rest |
| II | Reference tire model | PhysX closed form read off the wheel configuration |
| III | ROS 2 interface | `docs/CARLA_CUSTOM_BRIDGE.md` |
| IV | The two benchmark configurations | `benchmark_runner/scenarios.yaml`; the two parameter-file pairs |
| V | Cross-run comparison | `graphs/comparison/comparison_summary.csv` |
| VI | Baseline pipeline vs. this work, axis by axis | this study |
| VII | Peak friction vs. excitation, synthetic brush plant | synthetic-plant sweep |
| VIII | S4D identification cost | end-to-end timing on a synthetic 30 s lap |

Authoritative numbering comes from `grep newlabel main.aux`, not from this
table.

---

## Fig. 7 — `identifiability_sweep.pdf` — Rollout-speed identifiability sweep  ✅ DONE

**Type:** two-panel line plot with error bars. Vector PDF, included by
`sections/experiments.tex` §VI-B at `\columnwidth`, beside Table V
(`tab:sweep`).

**Generator:** `make_identifiability_sweep.py`. It **extends**
`On-Track-SysID/test/test_pacejka_identifiability.py` — that module owns the
rollout (`_rollout`, i.e. `simulated_data_gen()` with the residual network
zeroed), the vehicle dict (`_vehicle`) and the bound test (`_on_bound`). Do not
fork them; the whole point is that the figure and the regression test exercise
the same code path.

```bash
python3 paper/figures/make_identifiability_sweep.py     # ~200 s, no ROS, no torch
```

Writes `identifiability_sweep.csv` (one row per speed × true D × noise seed),
`identifiability_sweep_summary.csv` (the numbers in Table V) and
`identifiability_sweep.pdf`. Needs only numpy, scipy, yaml and matplotlib, so
it runs in the bare system python.

**Grid:** rollout speed {4, 6, 8, 10, 13, 15, 20} m/s × true `D`
{0.70, 1.05, 1.40, 1.80} × 5 noise seeds, unregularised
(`pacejka_prior_weight: 0`), started from the true coefficients,
`num_starts: 8`.

**Why the repetition is over noise and not over the solver:** the noise-free fit
is exactly seed-invariant here — five multi-start seeds return the same
coefficients to four decimal places, and so does a jitter width seven times
wider. That measurement *is* the "not a solver problem" claim, so it is stated
in the text rather than hidden. The intervals therefore come from Gaussian
noise injected on the rolled-out states at the two CARLA runs' own one-step
persistence RMSE, σ_vy = 0.010 m/s and σ_ω = 0.006 rad/s, read from
`graphs/comparison/comparison_summary.csv`.

**If the shipped tire, bounds or rollout change:** re-run it. The numbers in
Table V are transcribed from `identifiability_sweep_summary.csv` and there is
no build step that keeps them in step automatically.

---

## Results figures — format

`benchmark_runner/compare_scenarios.py::Comparison.save` writes a **vector PDF**
next to each PNG. `sections/experiments.tex` includes the comparison figures
**without an extension**, so `\DeclareGraphicsExtensions{.pdf,...}` picks the
PDF when it exists and falls back to the PNG until the next
`make compare_benchmark_scenarios`. Keep it that way: at IEEE single-column
width the 150 dpi rasters were ~257 dpi effective, under the 300 dpi floor.
`fig2.png` is a screenshot and stays a raster.
