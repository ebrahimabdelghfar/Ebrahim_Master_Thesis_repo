# Figure and table production guide

Each entry gives: the data source, the plot type, the axes, and a generation
recipe. Replace the corresponding `\placeholderbox{...}` with
`\includegraphics[width=\columnwidth]{<name>}` once the artwork exists. Export
PDF for vector plots, PNG only for screenshots.

Figures 1–3 are declared in `sections/method.tex` so they float beside the
Method text; figures 4–12 are in `sections/figure_placeholders.tex`. Numbering
follows declaration order, so keep the two in sync if you move a figure.

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

## Fig. 1 — `pipeline.pdf` — Identification pipeline

**Type:** block diagram with a feedback loop.

Stages: recorded lap → residual NN training → corrected model → synthetic
steering sweep → steady-state force recovery → bounded Pacejka fit → new
nominal model (loop back), with an exit arrow through the admissibility gate
to the MPC. Highlight the four contributions: rollout speed selection,
sweep bound `1.5·P99(|δ|)`, frozen regularisation target, derived-quantity
gate.

---

## Fig. 2 — `fig2.png` — Formula AI vehicle in CARLA  ✅ DONE

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

## Fig. 3 — `bridge_architecture.tex` — Carla_ASU_Bridge architecture  ✅ DONE

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

## Fig. 4 — `raceline.pdf` — The optimised raceline  ✅ DONE

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

## Fig. 5 — `telemetry.pdf` — Example telemetry

**Type:** 6-panel stacked line plot, shared x-axis (time, 0–30 s).

| Panel | y-axis | Series |
|---|---|---|
| a | `v_x` [m/s] | odom twist.linear.x |
| b | `v_y` [m/s] (left), `ω` [rad/s] (right) | odom twist.linear.y, twist.angular.z |
| c | `δ` [rad] | commanded (`/drive`) vs achieved (`feedback/steering_angle`, deg→rad) |
| d | `a_x`, `a_y` [m/s²] | `feedback/imu` linear_acceleration |
| e | `F_y` per axle [N] | measured (sum of FL+FR, RL+RR from `tire_forces`) vs Eq. (10) recovery |
| f | `α` per axle [rad] | measured (`tire_forces.slip_angle`) vs Eq. (3) |

```python
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv("sysid_export.csv")           # On-Track-SysID CSV export
fig, ax = plt.subplots(6, 1, sharex=True, figsize=(6.8, 8.0))
ax[0].plot(df.t, df.vx);                       ax[0].set_ylabel(r"$v_x$ [m/s]")
ax[1].plot(df.t, df.vy);                       ax[1].set_ylabel(r"$v_y$ [m/s]")
ax[1].twinx().plot(df.t, df.omega, "C1")
ax[2].plot(df.t, df.delta_cmd, label="commanded")
ax[2].plot(df.t, df.delta_meas, label="achieved"); ax[2].legend(ncol=2, fontsize=7)
# ... panels d-f likewise
ax[-1].set_xlabel("time [s]")
fig.tight_layout(); fig.savefig("telemetry.pdf")
```

Annotate panel (c) with the measured achieved/commanded ratio (0.76).

---

## Fig. 6 — `mu_reach.pdf` — Reachable friction  ★ central explanatory figure

**Type:** line plot, no measured data required — this is Eq. (11).

- x: rollout speed `v` ∈ [2, 20] m/s
- y: `μ_reach = v²·δ_sweep/(g·L)`, clipped for display at 3
- Solid curves: full scale, `L = 1.5334 m`, one per
  `δ_sweep ∈ {0.10, 0.20, 0.34, 0.40} rad`
- Dashed curves: 1:10 scale, `L = 0.32 m`, same `δ_sweep` set
- Horizontal line at `μ = 1.0` (measured plant peak), labelled
- Shaded vertical band over [2, 4] m/s = the inherited rollout clip
- Marker at the corrected operating point (measured lap speed, ≥ 7 m/s)

```python
import numpy as np, matplotlib.pyplot as plt
g = 9.81
v = np.linspace(2, 20, 400)
fig, ax = plt.subplots(figsize=(3.4, 2.6))
for L, ls, lab in [(1.5334, "-", "full scale"), (0.32, "--", "1:10")]:
    for d in (0.10, 0.20, 0.34, 0.40):
        ax.plot(v, np.minimum(v**2 * d / (g * L), 3.0), ls,
                label=f"{lab}, $\\delta$={d}" if d == 0.34 else None)
ax.axhline(1.0, color="k", lw=0.8)
ax.axvspan(2, 4, alpha=0.15, color="C3")
ax.set_xlabel("rollout speed [m/s]"); ax.set_ylabel(r"$\mu_{\rm reach}$")
ax.legend(fontsize=7); fig.tight_layout(); fig.savefig("mu_reach.pdf")
```

---

## Fig. 7 — `excitation.pdf` — Excitation and identifiability

**Type:** 2 panels.

(a) **Histograms** of `α_f`, `α_r` entering the fit, before (4 m/s rollout)
and after (measured-speed rollout). Overlay a vertical line at the reference
tire's peak-force slip angle, `α_peak = argmax F_y` from Table III.

(b) **Scatter + fitted curve** of `(α, F_y)` on the synthetic data, before and
after. The "before" panel should visibly show all data on the initial linear
ramp.

Produce by running the pipeline twice with
`pacejka_rollout.speed_min: 4.0` and `7.0` and dumping
`simulated_data_gen`'s output.

---

## Fig. 8 — `iterations.pdf` — Coefficient trajectories

**Type:** 2×4 grid of line plots (front/rear × B, C, D, E).

- x: co-identification iteration, 1…6
- y: coefficient value
- Two series: (i) unbounded sweep + tracking regularisation target,
  (ii) bounded sweep + frozen target
- Dashed horizontal lines: `PACEJKA_BOUNDS` lower/upper per coefficient, and
  the reference value from Table III

Log the per-iteration coefficients from `nn_train` (already printed when
`log_per_iteration: true`) into a CSV.

---

## Fig. 9 — `tire_curves.pdf` — Identified tire models  ★ primary accuracy claim

**Type:** 2 panels (front, rear), line plot over scatter.

- x: `α` ∈ [0, 0.4] rad; y: `F_y` [N] at the static axle load
- Reference fit (Table III), degenerate baseline fit, corrected fit as a
  shaded band over three runs
- Underneath: raw `(α, F_y)` scatter from `feedback/tire_forces`, alpha-blended

Parallels Fig. 6 of the baseline paper; use the same visual grammar so the
comparison is immediate.

---

## Fig. 10 — `one_step.pdf` — One-step prediction

**Type:** 2 time-series panels + inset residual histograms.

Measured vs predicted `v_y` and `ω` over 10 s of held-out data; annotate RMSE
and R². Source: `/estimated_state`, `/sensor_state`, `/estimation_error`
published by `on_track_sys_id`, or the `estimation_benchmark` node's
`vy_1step_metrics` / `omega_1step_metrics`.

---

## Fig. 11 — `demand_vs_grip.pdf` — Regenerated speed profile  *(pending)*

**Scope narrowed.** The "demand vs available grip" comparison this entry
originally specified is now Fig. 4(c), produced from the real trajectory. What
remains distinct — and still worth its own figure — is the *proposed fix*:
the profile after regeneration for the measured vehicle.

**Type:** 2 stacked line plots over arc length `s`, same axes as Fig. 4(c) so
the two read as before/after.

- Top: current demand (reproduce Fig. 4c for continuity), peak 12.00 m/s².
- Bottom: demand after regenerating with `m = 269.6 kg`, `μ_peak = 1.0` and
  `v_max ≈ 13–14 m/s`, against the same measured ceiling.

Verified reference points for the caption (recompute rather than copy):

| speed cap | peak demand | fraction of lap over the measured ceiling |
|---|---|---|
| 13 m/s | 9.05 m/s² (0.92 g) | 0.0 % |
| 14 m/s | 10.50 m/s² (1.07 g) | 0.0 % |
| 15 m/s | 12.00 m/s² (1.22 g) | 0.1 % |

A naive speed cap is a *lower bound* on what regeneration achieves — the
optimiser will also reshape the line. Produce the real thing by re-running
`main_globaltraj.py` with a corrected `racecar.ini` and `ggv.csv`, not by
clipping the existing CSV; the table above only bounds the expected result.

---

## Fig. 12 — `stiffness.pdf` — Cornering stiffness comparison

**Type:** grouped bar chart. Groups: simulator ground truth, degenerate fit,
command-referenced fit, achieved-steering fit, corrected fit. Bars: `C_f`,
`C_r` [kN/rad]. Annotate `l_f·C_f` vs `l_r·C_r` (understeer condition) above
each group.

---

## Fig. 13 — `closed_loop.pdf` — Closed-loop tracking  *(pending)*

Blocked on raceline regeneration. When available: (a) cross-track error vs
time, shaded by active controller state from `manager/state`; (b) driven path
over the raceline. The `adaptive_controller_benchmark` package already exports
`tracking_error_timeseries.png`, `handover_transient.png` and
`metrics_summary_table.png` on shutdown — use those as the source.

---

## Fig. 14 — `baseline_vs_ours.pdf` — Baseline pipeline vs. this work  *(pending)*

**Type:** 3 panels, full text width. **Needs no recorded run and no trained
network** — it is analytic plus an offline round-trip through the repository's
own solver, with the residual network held at zero so the rollout
configuration is the only variable. Held back until the identification runs
behind Figs. 7–10 are collected, so that a measured operating point can be
marked on panel (a).

Generator: `paper/figures/make_baseline_comparison.py` (`python3
paper/figures/make_baseline_comparison.py`, writes `baseline_vs_ours.pdf`
next to itself and prints the round-trip table). It imports
`On-Track-SysID/src/helpers/solve_pacejka.py` directly, so the fit in the
figure is the fit the pipeline runs.

- (a) `μ_reach = v²·δ_sweep/(g·L)` vs rollout speed, full scale (`L = 1.5334`)
  and 1:10 (`L = 0.32`), with the baseline's [2, 4] m/s clip band, the
  corrected `≥ 7` m/s band, and the plant's measured `μ ≈ 1.0`.
- (b) Known tire in, fitted tire out: identified `D_f`, `D_r` vs rollout
  speed over [2, 20] m/s, with the `PACEJKA_BOUNDS` D rails at 0.4 and 2.0 and
  the true `D` marked. **Run both arms** — baseline configuration (single
  start, no Tikhonov term, the shipped prior with `D` below its own lower
  bound) and the corrected configuration (8 starts, `prior_weight = 0.05`,
  measured prior) — so the panel separates the excitation effect from the
  solver and prior corrections. The script currently plots the corrected arm
  only; add the baseline arm before publishing.
- (c) `(α_f, F_y,f)` of the synthetic sweep at 4 m/s and at 12 m/s over the
  true Magic Formula curve, with the peak-force slip angle marked.

Pairs with Table VII (`tab:baseline`, Section VI-D), which is the qualitative
axis-by-axis comparison; this figure is its quantitative half. Fig. 6
(`mu_reach.pdf`) is a subset of panel (a) — if both are produced, drop Fig. 6
or reduce it to the single `δ_sweep` used in the runs.

---

## Tables

| Table | Content | Source |
|---|---|---|
| I | Vehicle configured vs measured | `carla_interface_config.yaml`; `feedback/tire_forces` at rest |
| II | ROS 2 interface | `docs/CARLA_CUSTOM_BRIDGE.md` |
| III | Reference tire model | direct MF fit to `tire_forces` over a limit run |
| IV | Closed-loop consequence | offline MPC harness |
| V | Hypotheses and verdicts | this study |
| VI | Defects, causes, corrections | this study |
| VII | Baseline pipeline vs. this work, axis by axis | this study; pairs with Fig. 14 |
| **VIII (to add)** | Coefficients: baseline / corrected / reference, per axle, with derived `C_f`, `C_r`, `a_y^max`, `v_crit` | `SIM_pacejka.txt` across runs |
| **IX (to add)** | Validation metrics: RMSE and R² for `v_y`, `ω` at 1/5/10-step horizons; per-axle force RMSE vs `tire_forces` | `estimation_benchmark`, `tire_force_benchmark` |

**Optional robustness column for Table VIII:** replicate the baseline's noise
sweep by enabling `odometry.noise` in the bridge config and scaling the
standard deviations by η ∈ [0, 1.4] in steps of 0.2, 10 repetitions each.
