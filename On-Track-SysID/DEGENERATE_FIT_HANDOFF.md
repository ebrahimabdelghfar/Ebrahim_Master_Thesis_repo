# Handoff: On-Track-SysID identifies a physically impossible tire model

**Task for you: debug, fix, and test the Pacejka identification in `On-Track-SysID/`.**
It currently converges on a tire model whose peak grip is ~0.40 g for a vehicle that really has
~1.5 g. This is an identification/estimation problem, not a controller problem — the downstream
MPC has already been fixed and is not in scope.

---

## 1. Context

ROS 2 Humble workspace at `/home/ebrahim/Ebrahim_Master_Thesis_repo`. The plant is a CARLA
full-scale Formula-Student car (`vehicle.vehicle.asurt_fsai`), 240 kg, wheelbase 1.5335 m, PhysX
`tire_friction = 1.5`.

The adaptive stack (`make launch_adaptive_controll`) runs:

```
pure_pursuit ──┐
               ├─► adaptive_controller_manager ──► /drive ──► CARLA
mpc_path_tracking ─┘          ▲
                              │ mpc/update_params  (8 floats: Bf,Cf,Df,Ef,Br,Cr,Dr,Er)
                    On-Track-SysID  ◄── YOU ARE HERE
```

`On-Track-SysID` observes the car, fits a Pacejka Magic Formula per axle, and pushes the
coefficients into the model-based MPC at runtime. Everything downstream is only as good as this
fit.

---

## 2. Symptom and how it was found

Under `make launch_adaptive_controll` the car oscillated violently at the pure-pursuit → MPC
handover, once MPC took over and began accelerating.

The handover mechanics were investigated and **ruled out** (steering discontinuity at the switch
measured 0.0006–0.0026 rad; adding bumpless transfer or a steering blend changed nothing). The
cause was the tire set being pushed. Measured in a closed-loop harness on the real raceline
(`traj_race_cl.csv`), 20 s with the switch at t=5 s:

| controller model | max cross-track after switch | horizon stages with no steady-state solution |
|---|---|---|
| hand-set prior (D = 1.5) | **0.03 m** | 0 |
| **the identified set** at 27 m/s | **41.89 m** | **29 461** |
| the identified set at 15 m/s | 0.43 m | 0 |
| the identified set at 11.1 m/s | 0.16 m | 0 |

Lateral demand goes as `v²`, which is why a bad grip estimate is invisible at pure-pursuit's
11.1 m/s and catastrophic once MPC accelerates.

**A guard already exists downstream and is not what you should change.**
`mpc_path_tracking` now rejects any identification whose grip ceiling
`(Df·Fz_f + Dr·Fz_r)/m` is below the raceline's peak `max(vx²·|κ|)`. It currently rejects every
identification, which means the adaptive loop no longer adapts:

```
mpc/update_params REJECTED: identified tires give a peak lateral acceleration of
3.95 m/s^2 (0.40 g, Df=0.400 Dr=0.405) but the raceline demands 12.00 m/s^2 (1.22 g).
```

Your job is to make the fit good enough to pass that gate honestly. **Do not loosen or remove
the gate.**

---

## 3. The evidence of degeneracy

Current output, `On-Track-SysID/models/SIM/SIM_pacejka.txt`:

```yaml
C_Pf: [5.2874, 1.5536, 0.4,    1.0    ]   # B, C, D, E
C_Pr: [19.3236, 1.2,   0.4046, -0.1292]
```

Against `PACEJKA_BOUNDS` in `src/helpers/solve_pacejka.py:201`:

```python
PACEJKA_BOUNDS = ([4.0, 1.2, 0.4, -3.0], [20.0, 2.2, 2.0, 1.0])
#                       ^^^^  ^^^                ^^^^
```

- `Df = 0.4` — **exactly** the lower bound.
- `Dr = 0.4046` — effectively on it.
- `Cr = 1.2` — **exactly** the lower bound.
- `Br = 19.32` — just under its upper bound of 20.
- `Ef = 1.0` — **exactly** the upper bound.

Five of eight coefficients are on or against a rail. That is not a fit; it is the optimizer
running out of information and sliding to the box edge. That file's own comment says D
"should converge near 1.5, the PhysX wheel `tire_friction`".

Consequences for the model handed to the MPC:

| quantity | identified | plausible |
|---|---|---|
| peak lateral accel `(Df·Fz_f + Dr·Fz_r)/m` | **3.95 m/s² (0.40 g)** | ~14.7 m/s² (1.5 g) |
| front axle cornering stiffness `Fz_f·Bf·Cf·Df` | **4 012 N/rad** | ~35 000 N/rad |
| rear axle cornering stiffness | 10 632 N/rad | ~32 000 N/rad |

A 4 012 N/rad front axle means the model needs ~0.30 rad of slip to make 1 g at the front — far
beyond the 0.2793 rad steering limit. The model believes the car essentially cannot corner.

This is the **third** oscillation bug in this repo traced to railed Pacejka parameters
(see `.wolf/buglog.json`: `bug-mpc-oscillation-scale-mismatch`,
`bug-mpc-qp-fail-rk4-stiff-linearization`, `bug-mpc-adaptive-handover-lowgrip-tires`).

---

## 4. Four suspected causes, in the order I'd test them

### 4.1 The friction warm-start measures *utilized* friction over an explicitly *low-slip* mask

`src/helpers/friction_warmstart.py` estimates the initial guess for D as

```python
mu_samples = np.sqrt(a_x[mask] ** 2 + a_y[mask] ** 2) / g
return float(np.median(mu_samples)), n_used
```

where `mask` is `_low_slip_mask(...)` — `|α_f| ≤ 0.15`, `|α_r| ≤ 0.08` rad by default
(`params/pacejka_params.yaml → friction_warm_start`).

**This looks structurally wrong.** `D` is the *available* peak friction coefficient. What is
being measured is the *utilized* friction, `√(aₓ²+a_y²)/g` — and it is measured **only over
samples that were explicitly selected for being far from the limit**. On a gentle raceline at
11.1 m/s, utilization there is ~0.2–0.4 g. So the prior is guaranteed to land near 0.4 no matter
what the tires can actually do. The docstring even calls it "instantaneous friction
utilization" — utilization is a lower bound on capability, never an estimate of it.

The referenced papers (arXiv:2603.09399 camera-based prior; arXiv:2509.15423 friction-from-
acceleration) infer *available* friction; check whether that method requires near-limit
excitation and whether this low-slip adaptation preserves its validity. My reading is that it
does not.

Verify: log `mu_hat` (the node already prints
`Friction warm-start (finite_diff): mu_hat=… from N low-slip samples`) and compare against the
peak `√(aₓ²+a_y²)/g` over the *whole* buffer, unmasked.

### 4.2 The static initial guess for D is *below* its own lower bound

`params/pacejka_params.yaml`:

```yaml
pacejka_model:
  C_Pf_model: [7.1737, 1.5628, 0.3882, 0.5278]   #  D = 0.3882  <  0.4 lower bound
  C_Pr_model: [8.2885, 2.1091, 0.3752, 0.4036]   #  D = 0.3752  <  0.4 lower bound
```

`solve_pacejka.py:79` does `starts = [np.clip(base, lb, ub)]`, so **the optimizer starts exactly
on the D rail**, and `pacejka_solver.num_starts: 1` means there is no second start to escape from.

This matters more than it looks: the warm-start only applies to the **first** identification
(`on_track_sys_id.py:344`, `_is_first_identification`). Every subsequent retrain
(`reidentification_interval_s`, default 30 s) cold-starts from these static values again — so
every retrain begins pinned at D = 0.4.

These values look like leftovers from the F1TENTH RC car this package originally targeted;
`pacejka_ref` alongside them has D = 0.69/0.68, also far from 1.5.

Verify: set `C_P*_model[2]` to ~1.5 and `num_starts` to e.g. 10, rerun, see whether D moves.
If it converges back to 0.4 from a start of 1.5, the problem is the data (§4.3/§4.4), not the
initialisation.

### 4.3 At low slip, only the product `B·C·D` is identifiable — not D alone

Near zero slip the Magic Formula linearises to `Fy ≈ Fz·B·C·D·α`. If the data never approaches
the tire's peak, the cost surface is flat along the manifold `B·C·D = const` and the individual
coefficients are unidentifiable; the solver will park wherever the box stops it.

`analyse_tires` (`solve_pacejka.py:118`) keeps `|α_f| ≤ 0.5`, `|α_r| ≤ 0.2` rad, but that is only
a rejection filter — it does not create excitation. Pure pursuit driving a smooth raceline at a
fixed 11.1 m/s (`pure_pursuit.yaml: use_fixed_reference_speed: true`) produces small, weakly
varying slip angles.

Verify: histogram the `α_f`/`α_r` actually used in the fit, and plot the fitted curve against the
`(α, Fy)` scatter. If all the data sits on the initial linear ramp, D is not observable from it
and no solver change will help — the fix is excitation (a slalom/step-steer manoeuvre, or
identifying at higher speed where `a_y = v²κ` is larger), or fixing `B`/`C` from prior knowledge
and fitting only `D`.

### 4.4 Unverified: is `/odom`'s twist body-frame?

`analyse_tires` builds slip angles from `v_y`:

```python
alpha_f = -np.arctan((v_y + omega * l_f) / v_x) + delta
alpha_r = -np.arctan((v_y - omega * l_r) / v_x)
```

and the axle forces from a **steady-state** assumption (no `v̇_y`, no `ω̇`):

```python
F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
F_yr = m * l_f * v_x * omega / (l_r + l_f)
```

Two things to check that I could not (the car was stationary whenever I sampled `/odom`):

1. **Frame.** If the CARLA bridge publishes `twist.linear` in the *world* frame rather than the
   body frame, `v_y` is wrong, every slip angle is wrong, and the fit is garbage in a way that
   would plausibly produce exactly this "very soft tires" result. Drive in a straight line at an
   angle to the world x-axis and confirm `twist.linear.y ≈ 0`.
2. **Steady-state force model.** The "measured" force is fully determined by `v_x·ω` — it is not
   a force measurement at all. Transients (which is where the informative data lives) are
   misattributed. CARLA publishes `/sim/feedback/tire_forces`; consider validating the computed
   `F_yf`/`F_yr` against it. Note `/sim/feedback/steering_angle` (Float32) is also published and
   currently has **zero subscribers** — worth checking that the `δ` used in the fit (taken from
   the `/drive` *command*) matches the steering actually achieved, since the actuator lags.

---

## 5. Also worth fixing while you are in here

- **Bounds are duplicated and disagree.** `solve_pacejka.py:201` says `PACEJKA_BOUNDS` "MUST stay
  in sync with `adaptive_controller_manager.yaml`'s `tire_param_min/tire_param_max`". They do not:
  sysid's D floor is 0.4, the manager's is **0.2**. The manager is the looser of the two and
  would admit an even worse fit.
- **`l_wb` vs `l_f + l_r`.** `models/SIM/SIM_pacejka.txt` has `l_wb: 1.53229` while
  `l_f + l_r = 0.738142 + 0.795262 = 1.533404`. `analyse_tires` uses `l_wb` for the normal loads
  but `l_f + l_r` for the force split, so they should be identical. (A much larger version of this
  bug — `l_wb` = 2.1305 against a real 1.5334 — was biasing every `Fz` by 28% and was fixed on
  2026-08-22; this residual 0.07 % is cosmetic but should be made exact.)
  Also `l_r` is `0.795262` here vs `0.795362` in `mpc_path_tracking.yaml` — pick one.
- `pacejka_solver.method: trf` with `num_starts: 1`. `differential-evolution` is already
  implemented and is a global search — a useful control experiment to distinguish "solver stuck in
  a local minimum" from "the data does not identify D".

---

## 6. Acceptance criteria

1. A fresh identification produces **no coefficient on a bound**, in particular `Df`, `Dr` well
   inside `(0.4, 2.0)` and near CARLA's `tire_friction = 1.5`.
2. Grip ceiling `(Df·Fz_f + Dr·Fz_r)/m ≥` the raceline's peak `max(vx²·|κ|)` — i.e.
   `mpc/update_params` returns `ack=True`. Test directly:
   ```bash
   ros2 service call /mpc/update_params \
     adaptive_controller_interfaces/srv/IdentifiedParam \
     "{param_values: [Bf,Cf,Df,Ef,Br,Cr,Dr,Er]}"
   ```
   (the MPC must have received the raceline first — look for
   `loaded N raceline waypoints` in its log, or the gate silently no-ops).
3. Axle cornering stiffnesses `Fz·B·C·D` land in the tens of kN/rad, not ~4 kN/rad. The MPC logs
   these on every update: `model check: C_front=… C_rear=… -> understeering/OVERSTEERING`.
4. The identified model must not be *oversteering* below `limits.speed_max` — the MPC also rejects
   that (`v_crit` gate), and a previous fit failed exactly this way.
5. `make launch_adaptive_controll` completes the PP→MPC handover and accelerates without
   oscillation.
6. Add a regression test asserting no fitted coefficient sits on a bound, so this signature cannot
   silently return a fourth time.

---

## 7. Files

| File | Why |
|---|---|
| `On-Track-SysID/src/helpers/solve_pacejka.py` | the fit; `PACEJKA_BOUNDS` (:201), `analyse_tires` (:118), multi-start (:79) |
| `On-Track-SysID/src/helpers/friction_warmstart.py` | the D prior — §4.1, the prime suspect |
| `On-Track-SysID/params/pacejka_params.yaml` | static D init below its own bound (§4.2), solver settings, warm-start config |
| `On-Track-SysID/src/on_track_sys_id.py` | data collection (50 Hz, 30 s buffer), `_is_first_identification` (:344), param submission (:528) |
| `On-Track-SysID/src/helpers/train_model.py` | `nn_train` (:605), applies `warm_start_mu` |
| `On-Track-SysID/models/SIM/SIM_pacejka.txt` | current (bad) output + vehicle geometry |
| `adaptive_controller_manager/config/adaptive_controller_manager.yaml` | `tire_param_min/max`, out of sync |
| `.wolf/buglog.json` | full history — read `bug-mpc-adaptive-handover-lowgrip-tires` first |

**Project conventions:** this repo uses OpenWolf. Read `.wolf/STATUS.md` first, check
`.wolf/cerebrum.md` (especially Do-Not-Repeat) before writing code, and log what you find to
`.wolf/buglog.json`. Build with
`colcon build --packages-select on_track_sys_id` (the MPC side, if you touch it, is
`--packages-select mpc_path_tracking`; its 38 gtests currently pass).

---

## 8. One caveat on my own analysis

I did not observe a live identification run end-to-end — `/odom` was idle whenever I sampled it,
so §4.4 in particular is reasoning from the code, not measurement. §4.1 and §4.2 are read
directly off the source and the current output file and I am confident in them; §4.3 is the
standard identifiability argument and needs the slip-angle histogram to confirm. Start by
reproducing an identification with logging before changing anything.

---

## 9. Outcome (2026-08-22)

Debugged live against the running CARLA sim with **pure pursuit only, no MPC**, plus a stub
service standing in for `sysid/update_params`. Ground truth throughout is CARLA's own per-wheel
telemetry, `/sim/feedback/tire_forces` (`sim_manager_msgs/msg/TireForces`, available by sourcing
`/home/ebrahim/Carla_ASU_Bridge/install/ros_apps/setup.bash` *after* the workspace overlay):
per-wheel `slip_angle`, `lateral_force` and `normal_load` straight out of the simulator.

### What the measurements said about §4

| Hypothesis | Verdict |
|---|---|
| §4.1 warm-start measures utilised friction over a low-slip mask | **Confirmed structurally, but not the cause.** Measured utilisation is 0.044 median / 0.46 peak at 13 m/s and 0.63 peak at 21 m/s. Fixed anyway: p99 over the whole buffer, applied as a *floor* on the prior. |
| §4.2 static `D` init below its own lower bound | **Confirmed, secondary.** Fixed, and `num_starts` 1 → 8. |
| §4.3 only `B·C·D` identifiable at low slip | **Confirmed, and it is the mechanism** — but not for the reason in §4.3. See below. |
| §4.4.1 `/odom` twist frame | **Wrong.** twist is body-frame: corr(twist.y, body v_y) = 0.93, RMS error 0.09 m/s, vs 16.7 m/s against a world-frame interpretation. |
| §4.4.2 steady-state force model | **Good enough.** Against CARLA's own axle force on a normal lap: corr 0.993, slope 0.94. |
| §4.4.2 `δ` from the command vs achieved | **Confirmed and significant.** Achieved road-wheel angle is **0.76 × commanded**. |

### The actual root cause

`solve_pacejka()` never sees the recorded lap. `nn_train` trains the residual NN on the lap, then
`simulated_data_gen()` rolls out (nominal Pacejka + NN) through a **synthetic** steering sweep,
and the Pacejka fit runs on that rollout. The rollout speed was
`avg_vel = np.clip(mean(v_x), 2.0, 4.0)` — an F1TENTH leftover. The friction such a rollout can
reach is `v²·δ_max/(g·l_wb)` = **0.43** at 4 m/s, so the synthetic data never leaves the linear
ramp and `D` slides onto its bound. Reproduced offline with the NN zeroed: a `D = 1.5` tire rolled
out at 4 m/s fits back as `Df = 0.753 / Dr = 0.737` with three coefficients railed; the same tire
at 8–20 m/s fits back as `D = 1.40–1.61`, nothing railed.

Five further defects were found while verifying the fix; all six are in `.wolf/buglog.json` as
`bug-sysid-*`, and the changes are listed in `.wolf/STATUS.md`.

### Measured tire curve of this vehicle

Fitted to CARLA's telemetry over a limit-excitation run (α to 0.37 rad, axle μ 1.05, 1.50 g):

```yaml
C_Pf: [7.076, 1.346, 1.009, -2.000]   # RMSE 135 N
C_Pr: [7.873, 1.383, 1.002, -1.024]   # RMSE  85 N
```

`D ≈ 1.0`, **not** 1.5: `tire_friction: 1.5` is the PhysX wheel material coefficient, while the
realised peak axle μ on this road surface measures 1.00–1.05. Also measured: mass **269.6 kg** at
rest (2644.5 N of wheel load) against the 240 kg in `carla_interface_config.yaml`, and axle
cornering stiffness **≈ 12.1·F_z per rad** (17198 / 16516 N/rad front / rear).

### Against the acceptance criteria

1. **Met.** Three consecutive live identifications, no coefficient on a bound, `Df/Dr` 1.05–1.18.
2. **Not met, and the gate is right.** The model now reports a grip ceiling of 1.02–1.16 g; the
   measured car does ~1.0 g steady state; `traj_race_cl.csv` demands 1.22 g. The gate was left
   untouched. **The raceline is what is infeasible** — its tightest corner (κ = 0.0536 1/m) allows
   14.4 m/s at the measured grip, and capping the profile at 13 m/s brings peak demand to 0.92 g.
3. **Met.** `C_front ≈ 21 kN/rad`, `C_rear ≈ 20 kN/rad` (CARLA's own: 17–19 kN/rad), against the
   4 kN/rad of the old fit.
4. **Met.** Understeering on every run (`l_f·C_f` < `l_r·C_r`), matching the measured plant.
5. **Not run** — blocked on criterion 2; the MPC will keep rejecting until the raceline is
   regenerated.
6. **Met.** `On-Track-SysID/test/test_pacejka_identifiability.py`, 8 tests, wired into
   `colcon test`.
