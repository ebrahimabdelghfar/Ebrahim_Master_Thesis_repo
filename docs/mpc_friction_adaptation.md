# Friction adaptation in the MPC

How `mpc_path_tracking` changes its behaviour when the road friction changes. The identification side is documented in docs/pacejka_identification.md; this file covers only what the controller does with the result.

Code: [grip_limits.hpp](mpc_path_tracking/include/mpc_path_tracking/grip_limits.hpp), [mpc_node.cpp](mpc_path_tracking/src/mpc_node.cpp), [reference_trajectory_handler.cpp](mpc_path_tracking/src/reference_trajectory_handler.cpp). Parameters: the `limits` and `mu_fast` blocks of [mpc_path_tracking.yaml](mpc_path_tracking/config/mpc_path_tracking.yaml).

## The problem

The MPC solves a QP against a reference speed profile. The profile is planned once from the raceline curvature, against a fixed lateral acceleration cap. If the road gets slippery, the cap is a lie: the reference still asks for corner speeds the tires cannot hold, the car understeers into the apex, and at a trail-braked apex the rear axle lets go. Making the *model* adaptive is not enough, because the reference the model tracks was never told the surface changed.

So everything below moves one number: the lateral acceleration the reference profile is allowed to plan against. When friction drops, that number drops, the profile is recut, and the car slows down before the corner rather than in it.

## Two inputs, two rates

| Source | Topic / service | Rate | What it gives |
|---|---|---|---|
| Pacejka coefficient set | `mpc/update_params` | ~1 per 30 s | full `B, C, D, E` per axle |
| Fast friction estimate | `/sysid/friction` | 1 Hz | `mu`, `sigma_mu`, `valid` |

The coefficient set comes from the On-Track-SysID pipeline [1]. It is accurate but slow, because a fit needs a buffer of excited driving.

The fast estimate is a Fiala brush model [2,3] fitted by least squares over a 6 s trailing window, per axle:

```
F_y = mu * F_z * (1 - (1 - xi)^3),    xi = clamp( C_alpha * |tan(alpha)| / (3 * mu * F_z), 0, 1 )
```

Fitting `(C_alpha, mu)` jointly recovers `mu` before the tire is fully sliding, which is what makes a 1 Hz estimate possible at all [4,5]. The channel is optional: with nothing publishing, the controller runs on the identified `D` alone and says nothing about it.

## 1. Grip ceiling

An axle's peak lateral force is `D * F_z`, so with static load split the car's peak lateral acceleration is

```
a_grip = ( |D_f| * F_zf + |D_r| * F_zr ) / m,     F_zf = m*g*l_r/L,  F_zr = m*g*l_f/L
```

This is the number the fast estimate scales between coefficient sets:

```
r = clamp( mu / mu_anchor, 0.4, 1.2 ),     target = a_grip * r
```

`mu_anchor` is the fast `mu` at the moment the last coefficient set was accepted. Anchoring is what stops the same friction drop being counted twice, once inside `D` and once again in the ratio.

## 2. The binding axle, not the sum

`a_grip` above adds the two axles, which is only reachable when `D_f == D_r` and there is no load transfer. In steady state, moment balance pins *both* axles to `F_y/F_z = a_y/g`, so a surplus at one axle can never cover a deficit at the other. Braking at `decel_max` moves `m*a_x*h_cg/L` off the rear — exactly the margin a trail-braked apex spends. The lateral cap therefore uses the weaker axle [6, §2]:

```
a_axle(a_x) = min( D_f * (g*l_r - a_x*h_cg)/l_r ,  D_r * (g*l_f + a_x*h_cg)/l_f )
axle_ratio  = a_axle(-decel_max) / a_grip
```

`axle_ratio` is carried as a scalar so that the rate limiter and the uncertainty tightening below act on one quantity.

## 3. Utilization: shape and uncertainty

An axle only reaches `D` at its peak slip angle. The identified tire is much softer than the startup reference (`B` about 5.1/5.4 against 10, `C` about 1.45 against 1.9), so its peak sits roughly twice as far out in slip, and a 20 Hz loop behind a drivetrain lag cannot chase a peak that has moved. The configured utilization is derated by how far the peak moved:

```
shape = min( 1, alpha*_ref / alpha*_id )    per axle, take the smaller
util  = grip_utilization * shape
```

`alpha*` is found by scanning the Magic Formula, not by the closed form, because `E > 1` makes the implicit equation non-monotone and identified sets do land there.

The fit's own uncertainty then tightens the margin further, in the spirit of cautious MPC [7]:

```
util <- clamp( util * (1 - k * sigma_mu / mu), util_min, util )
```

## 4. The cap, and the recut reference

```
a_y,max = min( lateral_accel_max, a_ceiling * util * axle_ratio )
v_ref(s) = sqrt( a_y,max / |kappa(s)| )
```

The profile is then forward/backward swept for longitudinal reachability, as in [8]. The longitudinal budget is coupled to the lateral demand by a friction ellipse [6, §13]:

```
a_lon,max = a_lon,cap * sqrt( 1 - (a_y / a_y,max)^2 )
```

so a corner does not get braked at the full straight-line rate.

## 5. Critical speed cap

A low-grip fit can be oversteering. Above the linear single-track critical speed

```
v_crit = L * sqrt( C_f * C_r / ( m * (l_f*C_f - l_r*C_r) ) ),    C = F_z * B * C * D
```

every horizon stage linearized there is open-loop unstable, and over `N` stages the QP inherits that as `rho^N` [6, §3]. The reference is capped at `0.9 * v_crit` instead. This used to be a rejection, which was wrong: under a decaying surface `v_crit` falls with grip, so rejecting would have frozen the controller on the last grippy model for the rest of the run. A set is refused only if the cap falls below `critical_speed_floor`, which describes a car that cannot drive the track at all.

## 6. Asymmetric rate limit

Grip that is gone is gone now; grip that has come back is a claim the next fit has yet to confirm.

```
ceiling <- target                                   if target <= ceiling
ceiling <- min( target, ceiling + rise_rate * dt )   otherwise
```

Every route to the ceiling passes through this, a full coefficient set included.

## 7. Fallbacks

- **No coefficient set yet.** A measured `mu` stands in as `D_f = D_r = mu`, and goes through the same arithmetic. Before this, a cold start on a slippery surface planned against `lateral_accel_max` alone.
- **Fast estimate stale** (older than `timeout_s`): revert to `D` alone, ratio back to 1.0, recovery still ramped.
- **Nothing ever published**: no warnings, no derate. This is the configuration the baseline benchmark arm runs in.
- **Achieved-acceleration ceiling.** The command limiter tracks what the drivetrain actually delivers and clamps the speed command to `v ± ceiling * reach_s`. That ceiling is floored at `accel_max` / `decel_max`; without the floor it decayed over a straight and could not be earned back, which cost about 7.7 s of braking authority at the next apex.

## 8. What each benchmark arm sees

The residual network architecture and the friction channels are independent. `apply_friction_warm_start` and the `pacejka_d_fixed` pin are applied in `nn_train` before it branches on `nn_architecture`, and the fast estimate is published by the node rather than by the model, so a baseline network and an S4D network hand the controller the same two quantities. What separates the arms is the parameter file they launch with, not the architecture.

**Baseline arm** (`nn_architecture: baseline`, `friction_warm_start.enable: false`, `mu_fast.enable: false`). Nothing is published on `/sysid/friction`, so `fast_mu_valid_` is never set: the ratio stays at 1.0, `sigma_mu` never tightens the utilization, and the grip ceiling changes only when a coefficient set is accepted. A friction change reaches the controller after up to `reidentification_interval_s`. Before the first set there is no ceiling at all and the reference is cut against `lateral_accel_max` alone, which is the cold start described in §7. The identification side leaves `D` free, and only the product `B*C*D` is observable from the rollout, so `D` walks to its upper bound while that product stays correct — the controller then plans against a grip ceiling the surface does not have. This arm is the control the comparison needs, and it is blind by construction.

**Warm-started arm** (`friction_warm_start.enable: true`, `mu_fast.enable: true`), with either architecture. Two channels reach the controller, anchored against each other so one friction drop is counted once.

The slow channel carries the surface level. The brush fit over the identification buffer gives a front-axle `mu`, which is written to both axles' `D` and held fixed for every iteration of the co-identification loop; an axle with no fit this cycle holds the `D` it came in with. The regulariser's prior is frozen from those values, so it targets the measurement rather than wherever the loop drifted. That `D` reaches the controller on `mpc/update_params`, which recomputes `a_grip`, `axle_ratio` and `shape`, and then re-anchors `mu_anchor` to the current fast `mu` and resets the ratio to 1.0.

The fast channel carries the change. Each 1 Hz fit is either ignored (`valid` false, normal on a straight, and the last ratio holds), used as `D_f = D_r = mu` when no coefficient set has arrived yet, or turned into the ratio `r` of §1 against the anchor. Its `sigma_mu` enters as margin rather than as grip, through the tightening of §3. Every resulting target passes the asymmetric rate limit of §6, and a stale estimate reverts to `D` alone.

Running the baseline network with the friction channels enabled is the ablation that separates the residual model from the grip handling, because it holds everything in this document constant.

## Measured effect

Offline, 0.5 %/s friction decay with a 30 s identification cycle. The metric is peak `a_lat / (mu*g)`; above 1.0 the car is asking for more than the surface has.

| arm | 0.5 %/s decay | 2 %/s decay | 40 % step |
|---|---|---|---|
| before | 1.120 FAIL (1 spin) | 8.04 FAIL (12 spins) | 0.958 PASS |
| + grip-scaled longitudinal limits | 1.120 FAIL | 12.54 FAIL | 0.957 PASS |
| + friction ellipse | 1.107 FAIL (0 spins) | 12.87 FAIL | 0.958 PASS |
| + shape utilization | **0.857 PASS** | **0.918 PASS** | 0.958 PASS |
| + fast mu (1 Hz) | **0.699 PASS** | 0.955 PASS | **0.707 PASS** |
| + sigma tightening | 0.761 PASS | 0.918 PASS | **0.657 PASS** |
| no fast mu published | 0.857 PASS | 0.918 PASS | 0.958 PASS |

The shape-utilization term carries the result on a decay; the fast channel is what buys margin on a step, where a 30 s coefficient cycle cannot react at all. The last two rows are not cleanly ordered, because a slower reference moves where the peak event happens.

Cold start at a constant `mu = 0.5`, with the binding-axle cap and the isotropic-`mu` fallback: spins 26 to 0, max `|e_y|` 277.07 m to 0.39 m, peak `a_lat/(mu*g)` 12.441 to 0.752, distance covered 3715 m to 5068 m. Spins are 0 across constant `mu` 0.4 / 0.5 / 0.7 / 1.0 and all three schedules. On a grippy surface the adaptation costs nothing: at `mu = 1.0` the lateral cap moves 6.50 to 5.82 m/s², still above the raceline's own 4.98 m/s² demand, so no waypoint clamps.

All of this is offline. It has not been run in CARLA.

## Parameters

| Parameter | Default | Role |
|---|---|---|
| `limits.grip_utilization` | 0.75 | base fraction of the ceiling the reference may use |
| `limits.grip_utilization_min` | 0.20 | floor after `sigma_mu` tightening |
| `limits.sigma_tighten_gain` | 1.0 | `k` in the tightening formula |
| `limits.friction_ellipse` | true | couple longitudinal budget to lateral demand |
| `limits.grip_rise_rate_per_s` | 0.5 m/s³ | recovery ramp; drops are instant |
| `limits.critical_speed_safety` | 0.9 | reference cap as a fraction of `v_crit` |
| `limits.critical_speed_floor` | 3.0 m/s | below this cap, the tire set is refused |
| `mu_fast.timeout_s` | 5.0 s | staleness before reverting to `D` alone |
| `mu_fast.ratio_bounds` | [0.4, 1.2] | clamp on `mu / mu_anchor` |

`limits.grip_utilization` and the yaml `tire.*` block are coupled: `shapeUtilization` derates the former by the peak-slip ratio against the latter. If 0.75 was tuned with an identified tire rather than with the startup prior, the term double-counts.

## References

1. O. Dikici, E. Ghignone, C. Hu, N. Baumann, L. Xie, A. Carron, M. Magno, M. Corno. Learning-Based On-Track System Identification for Scaled Autonomous Racing in Under a Minute. *IEEE RA-L* 10(2), 2025. doi:10.1109/LRA.2025.3527336
2. E. Fiala. Seitenkräfte am rollenden Luftreifen. *VDI-Zeitschrift* 96(29):973–979, 1954.
3. J. Svendenius, M. Gåfvert. A Brush-Model-Based Semi-Empirical Tire-Model for Combined Slips. SAE 2004-01-1064. doi:10.4271/2004-01-1064
4. M. Acosta, S. Kanarachos, M. Blundell. Road Friction Virtual Sensing: A Review of Estimation Techniques with Emphasis on Low Excitation Approaches. *Applied Sciences* 7(12):1230, 2017. doi:10.3390/app7121230
5. Y.-H. J. Hsu, S. M. Laws, J. C. Gerdes. Estimation of Tire Slip Angle and Friction Limits Using Steering Torque. *IEEE TCST* 18(4):896–907, 2010. doi:10.1109/TCST.2009.2031099
6. R. Rajamani. *Vehicle Dynamics and Control*, 2nd ed. Springer, 2012. doi:10.1007/978-1-4614-1433-9
7. L. Hewing, J. Kabzan, M. N. Zeilinger. Cautious Model Predictive Control Using Gaussian Process Regression. *IEEE TCST* 28(6):2736–2743, 2020. doi:10.1109/TCST.2019.2949757
8. A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann. Minimum Curvature Trajectory Planning and Control for an Autonomous Race Car. *Vehicle System Dynamics* 58(10):1497–1527, 2020. doi:10.1080/00423114.2019.1631455
9. H. B. Pacejka. *Tire and Vehicle Dynamics*, 3rd ed. Butterworth-Heinemann, 2012.
