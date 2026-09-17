// Offline closed-loop corner-entry harness.
//
// Links the real MpcController + AcadosMpcSolver + ReferenceTrajectoryHandler
// against a plant VehicleModel and replicates mpc_node::controlLoop (latency
// compensation, solve_on_new_odom, the drivetrain-tau speed command lead and
// one control period of transport delay).
//
// Writes a per-control-cycle trace to --out for analysis; the summary on stdout
// carries the metrics this class of bug needs: peak achieved lateral
// acceleration against the plant's grip ceiling, peak sideslip, a spin counter
// and distance covered.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "mpc_path_tracking/grip_limits.hpp"
#include "mpc_path_tracking/mpc_controller.hpp"
#include "mpc_path_tracking/reference_trajectory_handler.hpp"
#include "mpc_path_tracking/solver_interface.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

using namespace mpc_path_tracking;

namespace
{

struct Opts
{
  std::string csv{"traj_race_cl.csv"};
  std::string out{""};
  double psi_offset{M_PI_2};
  // controller config (mpc_path_tracking.yaml)
  int N{20};
  double dt_min{0.02};
  double dt_max{0.07};
  double horizon_distance{5.5};
  bool adaptive_distance{false};
  double distance_gain{1.0};
  double distance_curvature_gain{0.0};
  double control_rate{20.0};
  double q_ey{10.0}, q_epsi{5.0}, q_vx{100.0}, q_vy{0.0}, q_r{100.0};
  double r_steer{10.0}, r_accel{1.0};
  double rrate_steer{1000.0}, rrate_accel{1.0};
  double steer_max{0.2793};
  double steer_rate_max{1.0};
  double accel_max{5.71};
  double decel_max{3.71};
  double speed_max{31.0};
  double lateral_accel_max{6.5};
  double grip_util{0.0};          // 0 = off; else min(lateral_accel_max, ceiling*util)
  // controller tire model (yaml startup prior by default; --ctrl-* injects the
  // set On-Track-SysID actually pushes through mpc/update_params)
  double ctrl_Bf{10.0}, ctrl_Cf{1.9}, ctrl_Df{1.5}, ctrl_Ef{0.97};
  double ctrl_Br{10.0}, ctrl_Cr{1.9}, ctrl_Dr{1.5}, ctrl_Er{0.97};
  double tau_cfg{2.77};
  // plant
  double odom_rate{30.0};
  double tau_plant{2.77};
  double tau_plant_decel{-1.0};   // <0 = same as tau_plant (symmetric speed loop)
  bool tau_auto{false};           // replicate mpc_node's self-identification
  double tau_cfg_decel{-1.0};     // <0 = use tau_cfg for braking too
  bool tau_auto_split{false};     // identify accel and braking lags separately
  bool tau_ls{true};             // forgetting least squares instead of an EMA of ratios
  bool tau_sat_gate{false};       // drop samples taken at the plant's acceleration ceiling
  double tau_deadband{0.02};      // rewrite only on a change this big
  double tau_min_rewrite_s{1.0};  // minimum seconds between rewrites (0 = off)
  double tau_window{0.0};         // clamp the estimate to [cfg0/w, cfg0*w] (0 = off)
  double tau_vdot_min{0.0};       // with the gate on, also drop |vdot| below this*ceiling
  double tau_win_s{1.0};          // measure vdot over this window instead of one tick (0 = tick)
  bool cmd_antiwindup{true};     // clamp the command to what the plant can reach, freeze tau while clamped
  double ceiling_decay{0.995};   // per-cycle decay of the achieved-acceleration ceiling
  double ceiling_seed{0.0};       // initial ceiling (0 = the configured accel/decel limits)
  double tau_rate{1.2};           // max multiplicative change per rewrite (0 = unlimited)
  double mu{1.0};
  double plant_C{1.21};
  double plant_B{10.0};
  double plant_accel{5.71};
  double plant_decel{9.81};
  double duration{250.0};
  int delay_ticks{1};
  bool solve_on_new_odom{true};
  // Torque does not appear the instant the pedal moves. A second pole in series
  // with the speed lag is the minimum honest model of that, and it is what
  // supplies the phase lag a pure first-order lag cannot have. 0 = off.
  double plant_act_tau{0.0};
  double odom_noise{0.0};        // m/s, 1-sigma on the reported vx
  int seed{12345};               // odometry-noise realization
  // Candidate fix: low-pass the acceleration that gets multiplied by the
  // drivetrain lag. 0 = off (current shipped behaviour).
  double lead_lpf{0.0};
  // What the published speed is anchored to: "meas" = the predicted measured
  // speed (shipped behaviour), "ref" = the reference speed at the car's
  // predicted position.
  std::string cmd_base{"meas"};

  // ---- friction schedule (mirrors benchmark_runner/friction_schedule.py) ----
  std::string mu_schedule{"const"};   // const | decay | step
  double mu_decay_per_s{0.02};
  double mu_step_frac{0.6};
  double mu_step_t{60.0};
  double mu_floor{0.5};               // fraction of the nominal mu, not an absolute
  // ---- identification model -------------------------------------------------
  // The real estimators are covered by their own tests; what is under test here
  // is the controller's response to an estimate that lags, is noisy, and is
  // sometimes absent.
  double sysid_interval{30.0};        // s between 8-float coefficient sets (0 = never)
  double sysid_bias{1.0};             // multiplies the identified D, so 1.5 is a fit 50 % high
  double mu_fast_rate{1.0};           // Hz on sysid/friction (0 = nothing published)
  double mu_fast_window{6.0};         // s of trailing plant mu the fit averages
  double mu_fast_lag{0.25};           // s of detection delay on top of the window
  double mu_fast_noise{0.02};         // 1-sigma measurement noise on the fast mu
  double mu_fast_dropout_t0{-1.0};    // s; the estimator goes silent over [t0, t1)
  double mu_fast_dropout_t1{-1.0};
  // ---- ablation switches (mpc_node's limits.* block) ------------------------
  bool grip_longitudinal{true};       // derate accel/decel by the grip ceiling too
  bool axle_lateral{true};            // bind the lateral cap to the axle that saturates first
  // mpc_node has no ceiling until an estimate arrives, so --startup-ceiling 0 is
  // the node's real cold start and 1 is a run that begins already identified.
  bool startup_ceiling{true};
  bool fast_mu_absolute{true};        // before any coefficient set, the fast mu IS the ceiling
  bool friction_ellipse{true};
  bool shape_util{true};              // utilization from the identified tire shape
  double sigma_gain{1.0};
  double util_min{0.20};
  double grip_rise_rate{0.5};
  double crit_speed_safety{0.9};
};

// Plant peak friction at time t, as a fraction of the nominal mu.
double muAt(const Opts & o, double t)
{
  if (o.mu_schedule == "decay") {
    return o.mu * std::max(o.mu_floor, 1.0 - o.mu_decay_per_s * t);
  }
  if (o.mu_schedule == "step") {
    return t >= o.mu_step_t ? o.mu * o.mu_step_frac : o.mu;
  }
  return o.mu;
}

// Last occurrence wins, so a wrapper script can supply defaults and still let
// the caller override them from either side of the command line.
double arg(const std::vector<std::string> & a, const std::string & key, double def)
{
  double v = def;
  for (size_t i = 0; i + 1 < a.size(); ++i) {
    if (a[i] == key) {v = std::atof(a[i + 1].c_str());}
  }
  return v;
}

std::string argStr(const std::vector<std::string> & a, const std::string & key, std::string def)
{
  for (size_t i = 0; i + 1 < a.size(); ++i) {
    if (a[i] == key) {return a[i + 1];}
  }
  return def;
}

f1tenth_msgs::msg::WaypointArray loadRaceline(const std::string & path, double psi_offset)
{
  f1tenth_msgs::msg::WaypointArray msg;
  std::ifstream f(path);
  if (!f) {
    std::cerr << "cannot open " << path << "\n";
    std::exit(1);
  }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    for (char & c : line) {
      if (c == ';') {c = ' ';}
      if (c == ',') {c = ' ';}
    }
    std::istringstream ss(line);
    double s, x, y, psi, kappa, vx, ax;
    if (!(ss >> s >> x >> y >> psi >> kappa >> vx >> ax)) {continue;}
    f1tenth_msgs::msg::Waypoint w;
    w.s_m = s;
    w.x_m = x;
    w.y_m = y;
    w.psi_rad = std::atan2(std::sin(psi + psi_offset), std::cos(psi + psi_offset));
    w.kappa_radpm = kappa;
    w.vx_mps = vx;
    w.ax_mps2 = ax;
    msg.waypoints.push_back(w);
  }
  return msg;
}

VehicleParams carlaVehicle()
{
  VehicleParams v;
  v.mass = 240.0;
  v.Iz = 51.1;
  v.l_f = 0.738142;
  v.l_r = 0.795362;
  v.h_cg = 0.31538;
  return v;
}

TireParams controllerTire(const Opts & o)
{
  TireParams t;
  t.Bf = o.ctrl_Bf; t.Cf = o.ctrl_Cf; t.Df = o.ctrl_Df; t.Ef = o.ctrl_Ef;
  t.Br = o.ctrl_Br; t.Cr = o.ctrl_Cr; t.Dr = o.ctrl_Dr; t.Er = o.ctrl_Er;
  return t;
}

// Plant: peak axle mu and cornering stiffness as MEASURED off CARLA's own
// per-wheel telemetry (mu 1.00-1.05, C ~= 12.1*Fz per rad), not the
// controller's optimistic startup prior. `mu` is the schedule's value at the
// current time, so the surface changes under the running controller.
TireParams plantTire(const Opts & o, double mu)
{
  TireParams t;
  t.Bf = o.plant_B; t.Cf = o.plant_C; t.Df = mu; t.Ef = 0.97;
  t.Br = o.plant_B; t.Cr = o.plant_C; t.Dr = mu; t.Er = 0.97;
  return t;
}

// mpc_node's limits.* block, replayed against the harness's own switches so each
// term can be ablated independently. `budget` is what the reference may demand
// of the tires in any one direction.
struct GripState
{
  TireParams tire;                 // the controller's current coefficient set
  TireParams reference_tire;       // what grip_util was tuned against
  double pacejka_ceiling{0.0};
  double ceiling{0.0};
  double util_shape{1.0};
  double axle_lateral_ratio{1.0};
  double speed_cap{std::numeric_limits<double>::infinity()};
  double mu_anchor{0.0};
  double fast_mu{0.0}, fast_sigma{0.0}, fast_ratio{1.0};
  bool fast_valid{false};
  bool sysid_seen{false};          // a coefficient set has arrived, so the ratio has an anchor

  double budget(const Opts & o) const
  {
    if (!(ceiling > 0.0)) {return std::numeric_limits<double>::infinity();}
    double util = util_shape;
    if (fast_valid) {
      util = grip::applySigmaTightening(util, fast_mu, fast_sigma, o.sigma_gain, o.util_min);
    }
    return ceiling * util;
  }
};

void ingest(
  ReferenceTrajectoryHandler & ref, const f1tenth_msgs::msg::WaypointArray & raceline,
  const Opts & o, const GripState & g)
{
  const double a_grip = g.budget(o);
  ref.setSpeedLimit(std::min(o.speed_max, g.speed_cap));
  ref.setLateralAccelLimit(std::min(o.lateral_accel_max, a_grip * g.axle_lateral_ratio));
  ref.setFrictionEllipse(o.friction_ellipse);
  const double lon = o.grip_longitudinal ? a_grip : std::numeric_limits<double>::infinity();
  ref.setLongitudinalLimits(std::min(o.decel_max, lon), std::min(o.accel_max, lon));
  ref.setWaypoints(raceline);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> a(argv, argv + argc);
  Opts o;
  o.csv = argStr(a, "--csv", o.csv);
  o.out = argStr(a, "--out", o.out);
  o.N = static_cast<int>(arg(a, "--N", o.N));
  o.dt_min = arg(a, "--dt-min", o.dt_min);
  o.dt_max = arg(a, "--dt-max", o.dt_max);
  o.horizon_distance = arg(a, "--horizon-distance", o.horizon_distance);
  o.adaptive_distance = arg(a, "--adaptive-distance", o.adaptive_distance ? 1 : 0) != 0;
  o.distance_gain = arg(a, "--distance-gain", o.distance_gain);
  o.distance_curvature_gain = arg(a, "--distance-curv-gain", o.distance_curvature_gain);
  o.control_rate = arg(a, "--rate", o.control_rate);
  o.q_ey = arg(a, "--q-ey", o.q_ey);
  o.q_epsi = arg(a, "--q-epsi", o.q_epsi);
  o.q_vx = arg(a, "--q-vx", o.q_vx);
  o.q_r = arg(a, "--q-r", o.q_r);
  o.r_steer = arg(a, "--r-steer", o.r_steer);
  o.r_accel = arg(a, "--r-accel", o.r_accel);
  o.rrate_steer = arg(a, "--rrate-steer", o.rrate_steer);
  o.rrate_accel = arg(a, "--rrate-accel", o.rrate_accel);
  o.steer_rate_max = arg(a, "--steer-rate-max", o.steer_rate_max);
  o.accel_max = arg(a, "--accel", o.accel_max);
  o.decel_max = arg(a, "--decel", o.decel_max);
  o.speed_max = arg(a, "--speed-max", o.speed_max);
  o.lateral_accel_max = arg(a, "--alat", o.lateral_accel_max);
  o.grip_util = arg(a, "--grip-util", o.grip_util);
  o.ctrl_Bf = arg(a, "--ctrl-Bf", o.ctrl_Bf);
  o.ctrl_Cf = arg(a, "--ctrl-Cf", o.ctrl_Cf);
  o.ctrl_Df = arg(a, "--ctrl-Df", o.ctrl_Df);
  o.ctrl_Ef = arg(a, "--ctrl-Ef", o.ctrl_Ef);
  o.ctrl_Br = arg(a, "--ctrl-Br", o.ctrl_Br);
  o.ctrl_Cr = arg(a, "--ctrl-Cr", o.ctrl_Cr);
  o.ctrl_Dr = arg(a, "--ctrl-Dr", o.ctrl_Dr);
  o.ctrl_Er = arg(a, "--ctrl-Er", o.ctrl_Er);
  o.tau_cfg = arg(a, "--tau-cfg", o.tau_cfg);
  o.odom_rate = arg(a, "--odom-rate", o.odom_rate);
  o.tau_plant = arg(a, "--tau-plant", o.tau_plant);
  o.tau_plant_decel = arg(a, "--tau-plant-decel", o.tau_plant_decel);
  o.tau_auto = arg(a, "--tau-auto", o.tau_auto ? 1 : 0) != 0;
  o.tau_cfg_decel = arg(a, "--tau-cfg-decel", o.tau_cfg_decel);
  o.tau_auto_split = arg(a, "--tau-auto-split", o.tau_auto_split ? 1 : 0) != 0;
  o.tau_ls = arg(a, "--tau-ls", o.tau_ls ? 1 : 0) != 0;
  o.tau_sat_gate = arg(a, "--tau-sat-gate", o.tau_sat_gate ? 1 : 0) != 0;
  o.tau_deadband = arg(a, "--tau-deadband", o.tau_deadband);
  o.tau_min_rewrite_s = arg(a, "--tau-min-rewrite-s", o.tau_min_rewrite_s);
  o.tau_window = arg(a, "--tau-window", o.tau_window);
  o.tau_vdot_min = arg(a, "--tau-vdot-min", o.tau_vdot_min);
  o.tau_win_s = arg(a, "--tau-win-s", o.tau_win_s);
  o.cmd_antiwindup = arg(a, "--cmd-antiwindup", o.cmd_antiwindup ? 1 : 0) != 0;
  o.axle_lateral = arg(a, "--axle-lateral", o.axle_lateral ? 1.0 : 0.0) != 0.0;
  o.startup_ceiling = arg(a, "--startup-ceiling", o.startup_ceiling ? 1.0 : 0.0) != 0.0;
  o.fast_mu_absolute = arg(a, "--fast-mu-absolute", o.fast_mu_absolute ? 1.0 : 0.0) != 0.0;
  o.ceiling_decay = arg(a, "--ceiling-decay", o.ceiling_decay);
  o.ceiling_seed = arg(a, "--ceiling-seed", o.ceiling_seed);
  o.tau_rate = arg(a, "--tau-rate", o.tau_rate);
  o.mu = arg(a, "--mu", o.mu);
  o.plant_accel = arg(a, "--plant-accel", o.plant_accel);
  o.plant_decel = arg(a, "--plant-decel", o.plant_decel);
  o.duration = arg(a, "--dur", o.duration);
  o.delay_ticks = static_cast<int>(arg(a, "--delay-ticks", o.delay_ticks));
  o.solve_on_new_odom = arg(a, "--solve-on-new-odom", o.solve_on_new_odom ? 1 : 0) != 0;
  o.plant_act_tau = arg(a, "--plant-act-tau", o.plant_act_tau);
  o.odom_noise = arg(a, "--odom-noise", o.odom_noise);
  o.seed = static_cast<int>(arg(a, "--seed", o.seed));
  o.lead_lpf = arg(a, "--lead-lpf", o.lead_lpf);
  o.cmd_base = argStr(a, "--cmd-base", o.cmd_base);
  o.mu_schedule = argStr(a, "--mu-schedule", o.mu_schedule);
  o.mu_decay_per_s = arg(a, "--mu-decay-per-s", o.mu_decay_per_s);
  o.mu_step_frac = arg(a, "--mu-step-frac", o.mu_step_frac);
  o.mu_step_t = arg(a, "--mu-step-t", o.mu_step_t);
  o.mu_floor = arg(a, "--mu-floor", o.mu_floor);
  o.sysid_interval = arg(a, "--sysid-interval", o.sysid_interval);
  o.sysid_bias = arg(a, "--sysid-bias", o.sysid_bias);
  o.mu_fast_rate = arg(a, "--mu-fast-rate", o.mu_fast_rate);
  o.mu_fast_window = arg(a, "--mu-fast-window", o.mu_fast_window);
  o.mu_fast_lag = arg(a, "--mu-fast-lag", o.mu_fast_lag);
  o.mu_fast_noise = arg(a, "--mu-fast-noise", o.mu_fast_noise);
  o.mu_fast_dropout_t0 = arg(a, "--mu-fast-dropout-t0", o.mu_fast_dropout_t0);
  o.mu_fast_dropout_t1 = arg(a, "--mu-fast-dropout-t1", o.mu_fast_dropout_t1);
  o.grip_longitudinal = arg(a, "--grip-longitudinal", o.grip_longitudinal ? 1 : 0) != 0;
  o.friction_ellipse = arg(a, "--friction-ellipse", o.friction_ellipse ? 1 : 0) != 0;
  o.shape_util = arg(a, "--shape-util", o.shape_util ? 1 : 0) != 0;
  o.sigma_gain = arg(a, "--sigma-gain", o.sigma_gain);
  o.util_min = arg(a, "--util-min", o.util_min);
  o.grip_rise_rate = arg(a, "--grip-rise-rate", o.grip_rise_rate);
  o.crit_speed_safety = arg(a, "--crit-speed-safety", o.crit_speed_safety);

  const VehicleParams veh = carlaVehicle();
  GripState grip_state;
  grip_state.tire = controllerTire(o);
  // mpc_node's reference tire is the yaml `tire.*` startup block, which
  // limits.grip_utilization was tuned against - NOT the identified set that
  // arrives later over mpc/update_params. Taking it from a default-constructed
  // Opts is what keeps --ctrl-* an override of the identified tire alone;
  // reading it from `o` would make shapeUtilization compare a tire with itself
  // and silently reduce the whole term to a no-op.
  grip_state.reference_tire = controllerTire(Opts{});
  if (o.grip_util > 0.0 && o.startup_ceiling) {
    grip_state.pacejka_ceiling = grip::gripCeiling(grip_state.tire, veh);
    grip_state.ceiling = grip_state.pacejka_ceiling;
    grip_state.sysid_seen = true;
    grip_state.axle_lateral_ratio = o.axle_lateral
      ? grip::axleGripCeiling(grip_state.tire, veh, -o.decel_max) / grip_state.pacejka_ceiling
      : 1.0;
    grip_state.util_shape = o.shape_util
      ? grip::shapeUtilization(grip_state.tire, grip_state.reference_tire, o.grip_util)
      : o.grip_util;
    grip_state.speed_cap = o.crit_speed_safety * grip::criticalSpeed(grip_state.tire, veh);
  } else if (o.grip_util > 0.0) {
    grip_state.util_shape = o.grip_util;
  }

  ReferenceTrajectoryHandler ref;
  const auto raceline = loadRaceline(o.csv, o.psi_offset);
  ingest(ref, raceline, o, grip_state);
  if (!ref.hasWaypoints()) {
    std::cerr << "no waypoints loaded\n";
    return 1;
  }

  MpcConfig cfg;
  cfg.N = o.N;
  const double control_period = 1.0 / o.control_rate;
  cfg.dt_min = std::max(o.dt_min, control_period);   // parameter_manager does this
  cfg.dt_max = std::max(o.dt_max, cfg.dt_min);
  cfg.horizon_distance_m = o.horizon_distance;
  cfg.adaptive_distance = o.adaptive_distance;
  cfg.distance_gain = o.distance_gain;
  cfg.distance_curvature_gain = o.distance_curvature_gain;
  cfg.cost.Q << o.q_ey, o.q_epsi, o.q_vx, o.q_vy, o.q_r;
  cfg.cost.Qf = cfg.cost.Q;
  cfg.cost.R << o.r_steer, o.r_accel;
  cfg.cost.Rrate << o.rrate_steer, o.rrate_accel;
  cfg.limits.steering_min = -o.steer_max;
  cfg.limits.steering_max = o.steer_max;
  cfg.limits.steering_rate_max = o.steer_rate_max;
  cfg.limits.accel_max = o.accel_max;
  cfg.limits.accel_min = -o.decel_max;
  cfg.limits.jerk_max = 1000.0;

  AcadosSolverSettings acados;
  acados.qp_solver = "PARTIAL_CONDENSING_HPIPM";
  acados.cond_N = 5;
  acados.iter_max = 1000;
  MpcController ctrl(
    VehicleModel(carlaVehicle(), controllerTire(o)),
    std::make_unique<AcadosMpcSolver>(cfg.N, acados), cfg);

  VehicleModel plant(carlaVehicle(), plantTire(o, muAt(o, 0.0)));

  // ---- Corner map, built from the CLAMPED reference profile ----------------
  // The symptom under investigation ("brakes too late / not enough") is a
  // per-corner property, so the run is scored corner by corner rather than by
  // lap-wide extrema: where the reference starts braking against where the car
  // does, and how much speed the car is still carrying at the apex.
  std::vector<double> ref_s, ref_v;
  for (const auto & w : raceline.waypoints) {
    const auto p = ref.nearestPoint(w.x_m, w.y_m);
    ref_s.push_back(p.s);
    ref_v.push_back(p.vx);
  }
  const double track_len = ref_s.back() + (ref_s.back() - ref_s[ref_s.size() - 2]);
  const size_t nw = ref_v.size();
  auto wrapIdx = [nw](long i) {return static_cast<size_t>(((i % (long)nw) + (long)nw) % (long)nw);};

  struct Corner
  {
    size_t apex_idx;
    double apex_s, apex_v, ref_brake_s, ref_entry_v;
    // filled in during the run
    double car_v_at_apex{-1.0}, car_brake_s{-1.0}, car_entry_v{0.0};
    double peak_alat{0.0}, peak_beta{0.0};
  };
  std::vector<Corner> corners;
  const long W = 20;                       // +-40 m window at 2 m spacing
  for (size_t i = 0; i < nw; ++i) {
    bool is_min = true;
    double vmax = ref_v[i];
    for (long d = -W; d <= W; ++d) {
      const double vj = ref_v[wrapIdx((long)i + d)];
      if (vj < ref_v[i] - 1e-9) {is_min = false; break;}
      vmax = std::max(vmax, vj);
    }
    if (!is_min || vmax < ref_v[i] + 1.0) {continue;}    // need a real speed drop
    if (!corners.empty()) {
      const double gap = ref_s[i] - corners.back().apex_s;
      if (gap < 60.0) {continue;}                        // one apex per corner
    }
    // Reference brake onset: the upstream local speed maximum.
    long j = (long)i;
    double best_v = ref_v[i];
    long best_j = (long)i;
    for (long d = 1; d <= 150; ++d) {                    // up to 300 m upstream
      const size_t k = wrapIdx((long)i - d);
      if (ref_v[k] > best_v) {best_v = ref_v[k]; best_j = (long)k;}
      else if (ref_v[k] < best_v - 0.5) {break;}
    }
    (void)j;
    corners.push_back({i, ref_s[i], ref_v[i], ref_s[wrapIdx(best_j)], best_v});
  }

  // Per-control-cycle track of (arc length, speed) for the corner scoring.
  std::vector<double> log_s, log_v, log_alat, log_beta;

  // Start on the line at the reference speed.
  const auto start = ref.nearestPoint(raceline.waypoints[0].x_m, raceline.waypoints[0].y_m);
  State x;
  x << start.x, start.y, start.psi, std::min(start.vx, o.speed_max), 0.0, 0.0;

  const double sim_dt = 1.0 / 600.0;
  const int steps = static_cast<int>(o.duration / sim_dt);
  const double odom_period = 1.0 / o.odom_rate;

  State odom_state = x;
  double odom_stamp = 0.0, last_solved_stamp = -1.0, next_odom_t = 0.0, next_ctrl_t = 0.0;
  Input u_prev = Input::Zero();
  double last_solve_ms = 0.0;
  double speed_cmd = x(3), steer_cmd = 0.0;
  bool has_solved = false;

  std::vector<std::pair<double, double>> pending;   // (apply_time, value) delay queue
  double applied_steer = 0.0, applied_speed = x(3);
  std::vector<double> delayed_steer, delayed_speed, delayed_time;

  std::ofstream trace;
  if (!o.out.empty()) {
    trace.open(o.out);
    trace << "t,s,x,y,psi,vx,vy,r,delta,speed_cmd,u_accel,e_y,vx_ref,kappa_ref,a_lat,beta,solved,"
             "tau_cfg,tau_cfg_decel,mu_plant,mu_est,sigma_mu,fast_valid,ceiling,util,ay_max,"
             "ay_ratio,alpha_f,alpha_r\n";
  }

  // mpc_node::estimateDrivetrainTau replica (limits.drivetrain_tau_auto)
  double tau_cfg = o.tau_cfg, tau_estimate = 0.0;
  double tau_cfg_decel = o.tau_cfg_decel > 0.0 ? o.tau_cfg_decel : o.tau_cfg;
  double tau_estimate_decel = 0.0;
  // Tracked 95th percentile of |vdot| per direction, seeded at the planner's
  // own limit. Samples at that ceiling are the torque limiter, not the lag.
  double vdot_ceiling = o.ceiling_seed > 0.0 ? o.ceiling_seed : o.accel_max;
  double vdot_ceiling_decel = o.ceiling_seed > 0.0 ? o.ceiling_seed : o.decel_max;
  double ls_dd = 0.0, ls_dv = 0.0, ls_dd_decel = 0.0, ls_dv_decel = 0.0;
  const double tau_prior = o.tau_cfg, tau_prior_decel = tau_cfg_decel;
  double last_rewrite_t = -1e9, last_rewrite_t_decel = -1e9;
  int tau_rewrites = 0, tau_rewrites_decel = 0;
  double tau_peak = o.tau_cfg, tau_peak_decel = tau_cfg_decel;
  std::vector<double> hist_cmd, hist_meas;
  bool cmd_clamped = false;
  int clamp_in_window = 0;
  double ripple_sum = 0.0, ripple_max = 0.0;
  int ripple_n = 0;
  int tau_samples = 0, tau_samples_decel = 0;
  bool has_prev_speed_sample = false;
  double prev_speed_cmd = 0.0, prev_speed_meas = 0.0;

  std::mt19937 rng(static_cast<unsigned>(o.seed));
  std::normal_distribution<double> noise(0.0, 1.0);
  double a_actual = 0.0;        // plant acceleration behind the actuator pole
  double lead_accel = 0.0;      // low-passed acceleration feeding the tau lead

  double max_ey = 0.0, peak_alat = 0.0, peak_beta = 0.0, distance = 0.0;
  int spins = 0, failures = 0, cycles = 0;
  // Horizon-length scoring: dt is what the adaptive preview law moves, solve
  // time is what it costs, and RMS e_y is the accuracy max|e_y| alone hides.
  double dt_sum = 0.0, dt_min_seen = 1e9, dt_max_seen = 0.0;
  double solve_sum = 0.0, solve_max = 0.0;
  std::vector<double> solve_all;
  double ey_sq_sum = 0.0;
  int ey_n = 0;
  bool in_spin = false;
  int sign_flips = 0;
  double last_steer = 0.0;

  // ---- friction schedule + identification model ---------------------------
  double next_sysid_t = o.sysid_interval > 0.0 ? o.sysid_interval : 1e18;
  double next_fast_t = o.mu_fast_rate > 0.0 ? 1.0 / o.mu_fast_rate : 1e18;
  double last_ceiling_t = 0.0;
  double last_fast_t = 0.0, last_fast_publish_t = 0.0;
  std::normal_distribution<double> mu_noise(0.0, 1.0);
  // Mean and spread of the plant's mu over the window the fit would have seen.
  // The spread is what a short window costs in confidence, and it is the whole
  // reason sigma_mu grows across a step.
  auto fastFit = [&](double t, double & mu_hat, double & sigma) {
      constexpr int kSamples = 20;
      double sum = 0.0, sum_sq = 0.0;
      for (int i = 0; i < kSamples; ++i) {
        const double ti = std::max(
          0.0, t - o.mu_fast_lag - o.mu_fast_window * i / (kSamples - 1.0));
        const double m = muAt(o, ti);
        sum += m;
        sum_sq += m * m;
      }
      const double mean = sum / kSamples;
      const double var = std::max(0.0, sum_sq / kSamples - mean * mean);
      mu_hat = mean + o.mu_fast_noise * mu_noise(rng);
      sigma = std::sqrt(var + o.mu_fast_noise * o.mu_fast_noise);
    };
  double mu_plant = muAt(o, 0.0);
  double worst_ay_ratio = 0.0, worst_ay_ratio_t = 0.0;
  double peak_alpha_f = 0.0, peak_alpha_r = 0.0;
  int fast_publishes = 0, sysid_updates = 0;

  for (int k = 0; k < steps; ++k) {
    const double t = k * sim_dt;

    // The surface changes under the running controller; everything downstream
    // has to find out about it through the estimators, not by reading this.
    mu_plant = muAt(o, t);
    plant.setTireParams(plantTire(o, mu_plant));

    // Fast friction estimate on sysid/friction. Nothing is published at all
    // when the rate is 0 or during a dropout window, which is the
    // no-warm-start configuration the controller must still run in.
    if (t >= next_fast_t) {
      const double dt_fast = t - last_fast_t;
      last_fast_t = t;
      next_fast_t += 1.0 / o.mu_fast_rate;
      const bool dropped = o.mu_fast_dropout_t0 >= 0.0 &&
        t >= o.mu_fast_dropout_t0 && t < o.mu_fast_dropout_t1;
      if (!dropped) {
        double mu_hat = 0.0, sigma = 0.0;
        fastFit(t, mu_hat, sigma);
        ++fast_publishes;
        grip_state.fast_mu = mu_hat;
        grip_state.fast_sigma = sigma;
        grip_state.fast_valid = true;
        if (o.fast_mu_absolute && !grip_state.sysid_seen) {
          // Nothing to derate relative to yet, so the measured mu stands in as D.
          TireParams iso = grip_state.tire;
          iso.Df = mu_hat;
          iso.Dr = mu_hat;
          grip_state.pacejka_ceiling = grip::gripCeiling(iso, veh);
          grip_state.axle_lateral_ratio = o.axle_lateral
            ? grip::axleGripCeiling(iso, veh, -o.decel_max) / grip_state.pacejka_ceiling
            : 1.0;
          grip_state.fast_ratio = 1.0;
        } else {
          if (!(grip_state.mu_anchor > 0.0)) {grip_state.mu_anchor = mu_hat;}
          grip_state.fast_ratio = std::clamp(mu_hat / grip_state.mu_anchor, 0.4, 1.2);
        }
      } else if (grip_state.fast_valid && t - last_fast_publish_t > 5.0) {
        // mu_fast.timeout_s: revert to the identified D alone. The ceiling still
        // climbs back on the ramp, so losing the estimator never steps the
        // reference speed up.
        grip_state.fast_valid = false;
        grip_state.fast_ratio = 1.0;
      }
      if (o.grip_util > 0.0) {
        grip_state.ceiling = grip::rateLimitedCeiling(
          grip_state.ceiling, grip_state.pacejka_ceiling * grip_state.fast_ratio,
          o.grip_rise_rate, t - last_ceiling_t);
        last_ceiling_t = t;
        ingest(ref, raceline, o, grip_state);
      }
      if (!dropped) {last_fast_publish_t = t;}
    }

    // A full coefficient set, fitted over the whole identification cycle and
    // so anchored that far back in time.
    if (t >= next_sysid_t) {
      next_sysid_t += o.sysid_interval;
      const double mu_fit = o.sysid_bias * muAt(o, std::max(0.0, t - o.sysid_interval * 0.5));
      TireParams id = controllerTire(o);
      id.Df = mu_fit;
      id.Dr = mu_fit;
      ++sysid_updates;
      ctrl.setTireParams(id);
      grip_state.tire = id;
      if (o.grip_util > 0.0) {
        grip_state.pacejka_ceiling = grip::gripCeiling(id, veh);
        // Rate limited like every other route to the ceiling: a coefficient set
        // that claims more grip is a claim the surface has yet to confirm.
        grip_state.ceiling = grip::rateLimitedCeiling(
          grip_state.ceiling, grip_state.pacejka_ceiling, o.grip_rise_rate, t - last_ceiling_t);
        last_ceiling_t = t;
        grip_state.sysid_seen = true;
        grip_state.axle_lateral_ratio = o.axle_lateral
          ? grip::axleGripCeiling(id, veh, -o.decel_max) / grip_state.pacejka_ceiling
          : 1.0;
        grip_state.util_shape = o.shape_util
          ? grip::shapeUtilization(id, grip_state.reference_tire, o.grip_util)
          : o.grip_util;
        grip_state.speed_cap = o.crit_speed_safety * grip::criticalSpeed(id, veh);
        grip_state.mu_anchor = grip_state.fast_valid ? grip_state.fast_mu : 0.0;
        grip_state.fast_ratio = 1.0;
        ingest(ref, raceline, o, grip_state);
      }
    }

    if (t >= next_odom_t) {
      odom_state = x;
      if (o.odom_noise > 0.0) {odom_state(3) += o.odom_noise * noise(rng);}
      odom_stamp = t;
      next_odom_t += odom_period;
    }

    if (t >= next_ctrl_t) {
      next_ctrl_t += control_period;
      bool reused = false;
      if (o.solve_on_new_odom && has_solved && odom_stamp == last_solved_stamp) {
        reused = true;
      }
      if (!reused) {
        last_solved_stamp = odom_stamp;
        const double odom_age = std::clamp(t - odom_stamp, 0.0, 0.2);
        const double horizon_s =
          std::clamp(odom_age + control_period + last_solve_ms * 1e-3, 0.0, 0.25);
        const State x0 =
          horizon_s > 0.0 ? ctrl.predictState(odom_state, u_prev, horizon_s) : odom_state;
        const MpcOutput out = ctrl.computeCommand(x0, u_prev, ref);
        ++cycles;
        dt_sum += out.dt_used;
        dt_min_seen = std::min(dt_min_seen, out.dt_used);
        dt_max_seen = std::max(dt_max_seen, out.dt_used);
        solve_sum += out.solve_time_ms;
        solve_max = std::max(solve_max, out.solve_time_ms);
        solve_all.push_back(out.solve_time_ms);
        if (!out.solved) {
          ++failures;                       // fallback: hold_last
        } else {
          last_solve_ms = out.solve_time_ms;
          u_prev = out.u0;
          steer_cmd = out.u0(0);
          // The lag inversion multiplies an acceleration by ~tau (2.82 s at the
          // shipped config, 56x the control period). u0(1) is a feedback
          // quantity, so without the filter every cycle-to-cycle wobble in it
          // lands in the published speed multiplied by that factor. The filter
          // passes the DC value untouched - braking and post-handover
          // convergence both survive - and removes only what the plant, whose
          // own time constant is tau, could not follow anyway.
          lead_accel = o.lead_lpf > 0.0
            ? lead_accel + (out.u0(1) - lead_accel) * std::min(1.0, control_period / o.lead_lpf)
            : out.u0(1);
          const double tau_used = lead_accel < 0.0 ? tau_cfg_decel : tau_cfg;
          // A speed setpoint should be a reference, not an echo of the
          // measurement: anchoring on x0(3) feeds every bit of odometry noise
          // straight out to the actuator at unity gain, on top of the same
          // noise re-entering through u0(1) multiplied by tau.
          const double base = o.cmd_base == "ref" ?
            ref.nearestPoint(x0(0), x0(1)).vx : x0(3);
          const double prev_cmd_for_ripple = speed_cmd;
          speed_cmd = std::clamp(
            base + out.u0(1) * control_period + lead_accel * tau_used, 0.0, o.speed_max);
          // Anti-windup: a speed the plant cannot reach within one lag constant
          // is not a setpoint, it is windup - and the lag identified from it
          // measures the torque limiter instead of the lag.
          cmd_clamped = false;
          if (o.cmd_antiwindup) {
            const double reach = control_period + tau_used;
            const double hi = x0(3) + vdot_ceiling * reach;
            const double lo = x0(3) - vdot_ceiling_decel * reach;
            const double limited = std::clamp(speed_cmd, lo, hi);
            cmd_clamped = std::abs(limited - speed_cmd) > 1e-9;
            speed_cmd = limited;
          }
          if (has_solved) {
            const double d = std::abs(speed_cmd - prev_cmd_for_ripple) * 3.6;
            ripple_sum += d;
            ripple_max = std::max(ripple_max, d);
            ++ripple_n;
          }
          has_solved = true;
        }
        if (o.tau_auto) {
          hist_cmd.push_back(speed_cmd);
          hist_meas.push_back(x(3));
          if (cmd_clamped) {
            clamp_in_window = std::max(
              1, static_cast<int>(std::lround(std::max(o.tau_win_s, control_period) /
              control_period)));
          }
          const int win = o.tau_win_s > 0.0
            ? std::max(1, static_cast<int>(std::lround(o.tau_win_s / control_period))) : 1;
          const bool have_win = static_cast<int>(hist_meas.size()) > win;
          if (o.tau_win_s > 0.0 ? have_win : has_prev_speed_sample) {
            const size_t n_h = hist_meas.size();
            double drive = prev_speed_cmd - prev_speed_meas;
            double vdot = (x(3) - prev_speed_meas) / control_period;
            if (o.tau_win_s > 0.0) {
              // A 2.77 s lag moves the speed by 2% in one 0.05 s tick: the
              // single-tick difference is mostly odometry noise. Averaging the
              // command error over a window and differencing the speed across
              // the same window puts real signal on both sides of the ratio.
              double sum_drive = 0.0;
              for (size_t i = n_h - 1 - win; i + 1 < n_h; ++i) {
                sum_drive += hist_cmd[i] - hist_meas[i];
              }
              drive = sum_drive / win;
              vdot = (hist_meas[n_h - 1] - hist_meas[n_h - 1 - win]) / (win * control_period);
            }
            const double av = std::abs(vdot);
            const bool braking = drive < 0.0;
            double & ceiling = vdot > 0.0 ? vdot_ceiling : vdot_ceiling_decel;
            // Floored at the planner's own limit, as mpc_node's updateAccelCeiling is.
            const double ceiling_floor = vdot > 0.0 ? o.accel_max : o.decel_max;
            ceiling = std::max(ceiling_floor, std::max(o.ceiling_decay * ceiling, av));
            clamp_in_window = std::max(0, clamp_in_window - 1);
            const bool saturated = (o.tau_sat_gate && (av > 0.9 * ceiling || av < o.tau_vdot_min * ceiling)) ||
              (o.cmd_antiwindup && clamp_in_window > 0);
            if (std::abs(drive) > 0.5 && drive * vdot > 0.0 && av > 1e-3 && !saturated) {
              const double sample = std::clamp(drive / vdot, 0.0, 5.0);
              if (std::isfinite(sample) && sample > 0.0) {
                double & est = (o.tau_auto_split && braking) ? tau_estimate_decel : tau_estimate;
                int & n = (o.tau_auto_split && braking) ? tau_samples_decel : tau_samples;
                if (o.tau_ls) {
                  // Forgetting least squares on vdot = drive/tau: weights each
                  // sample by drive^2, so a near-zero vdot cannot dominate the
                  // way an average of drive/vdot ratios does.
                  double & sdd = (o.tau_auto_split && braking) ? ls_dd_decel : ls_dd;
                  double & sdv = (o.tau_auto_split && braking) ? ls_dv_decel : ls_dv;
                  sdd = 0.99 * sdd + drive * drive;
                  sdv = 0.99 * sdv + drive * vdot;
                  est = std::clamp(sdd / sdv, 0.0, 5.0);
                } else {
                  est = n > 0 ? 0.98 * est + 0.02 * sample : sample;
                }
                ++n;
                const bool dec = o.tau_auto_split && braking;
                double & cfg = dec ? tau_cfg_decel : tau_cfg;
                double & last_t = dec ? last_rewrite_t_decel : last_rewrite_t;
                int & rewrites = dec ? tau_rewrites_decel : tau_rewrites;
                if (o.tau_window > 0.0) {
                  const double prior = dec ? tau_prior_decel : tau_prior;
                  est = std::clamp(est, prior / o.tau_window, prior * o.tau_window);
                }
                // Rate limit: a lag constant that is one number today cannot be
                // triple that a second later, so a step that big is evidence of
                // a bad window, not of a changed drivetrain.
                if (o.tau_rate > 1.0) {
                  est = std::clamp(est, cfg / o.tau_rate, cfg * o.tau_rate);
                }
                if (n >= 50 && std::abs(est - cfg) > o.tau_deadband &&
                  t - last_t >= o.tau_min_rewrite_s)
                {
                  cfg = est;
                  last_t = t;
                  ++rewrites;
                  if (!o.tau_auto_split) {tau_cfg_decel = tau_cfg;}
                }
                tau_peak = std::max(tau_peak, tau_cfg);
                tau_peak_decel = std::max(tau_peak_decel, tau_cfg_decel);
              }
            }
          }
          prev_speed_cmd = speed_cmd;
          prev_speed_meas = x(3);
          has_prev_speed_sample = true;
        }
      }
      // one control period of transport delay per tick
      delayed_time.push_back(t + o.delay_ticks * control_period);
      delayed_steer.push_back(steer_cmd);
      delayed_speed.push_back(speed_cmd);

      {
        const auto np = ref.nearestPoint(x(0), x(1));
        const double e_y = (x(1) - np.y) * std::cos(np.psi) - (x(0) - np.x) * std::sin(np.psi);
        const double beta = std::atan2(x(4), std::max(x(3), 0.1));
        double trace_alpha_f = 0.0, trace_alpha_r = 0.0;
        plant.slipAngles(x, applied_steer, trace_alpha_f, trace_alpha_r);
        log_s.push_back(np.s);
        log_v.push_back(x(3));
        log_alat.push_back(std::abs(x(3) * x(5)));
        log_beta.push_back(std::abs(beta));
        if (o.out.size()) {
          trace << t << "," << np.s << "," << x(0) << "," << x(1) << "," << x(2) << ","
                << x(3) << "," << x(4) << "," << x(5) << "," << applied_steer << ","
                << applied_speed << "," << u_prev(1) << "," << e_y << "," << np.vx << ","
                << np.kappa << "," << x(3) * x(5) << "," << beta << ","
                << (has_solved ? 1 : 0) << "," << tau_cfg << "," << tau_cfg_decel << ","
                << mu_plant << "," << grip_state.fast_mu << "," << grip_state.fast_sigma << ","
                << (grip_state.fast_valid ? 1 : 0) << "," << grip_state.ceiling << ","
                << (grip_state.ceiling > 0.0 ? grip_state.budget(o) / grip_state.ceiling : 0.0)
                << "," << ref.lateralAccelLimit() << ","
                << std::abs(x(3) * x(5)) / (mu_plant * 9.81) << ","
                << trace_alpha_f << "," << trace_alpha_r << "\n";
        }
      }
      if (last_steer * steer_cmd < 0.0) {++sign_flips;}
      last_steer = steer_cmd;
    }

    while (!delayed_time.empty() && delayed_time.front() <= t) {
      applied_steer = delayed_steer.front();
      applied_speed = delayed_speed.front();
      delayed_time.erase(delayed_time.begin());
      delayed_steer.erase(delayed_steer.begin());
      delayed_speed.erase(delayed_speed.begin());
    }

    // Plant speed loop. "lag" is the idealised first-order lag; "pid" is CARLA's
    // own cascaded Ackermann controller, stepped at the server rate.
    const double err = applied_speed - x(3);
    const double tau_p = (err < 0.0 && o.tau_plant_decel > 0.0) ? o.tau_plant_decel : o.tau_plant;
    const double a_demand = std::clamp(err / tau_p, -o.plant_decel, o.plant_accel);
    // Optional second pole: the torque the pedal asks for arrives with its own
    // lag, which is the phase a single first-order lag does not have.
    if (o.plant_act_tau > 0.0) {
      a_actual += (a_demand - a_actual) * std::min(1.0, sim_dt / o.plant_act_tau);
    } else {
      a_actual = a_demand;
    }
    const double a_plant = a_actual;
    const State x_next = plant.integrateRk4(x, Input(applied_steer, a_plant), sim_dt);
    distance += std::hypot(x_next(0) - x(0), x_next(1) - x(1));
    x = x_next;

    const auto np = ref.nearestPoint(x(0), x(1));
    const double e_y = (x(1) - np.y) * std::cos(np.psi) - (x(0) - np.x) * std::sin(np.psi);
    max_ey = std::max(max_ey, std::abs(e_y));
    ey_sq_sum += e_y * e_y;
    ++ey_n;
    peak_alat = std::max(peak_alat, std::abs(x(3) * x(5)));
    // The pass criterion: what the car actually asked of the tires, against what
    // the surface could give AT THAT MOMENT. Above 1.0 the car is sliding.
    const double ay_ratio = std::abs(x(3) * x(5)) / (mu_plant * 9.81);
    if (ay_ratio > worst_ay_ratio) {worst_ay_ratio = ay_ratio; worst_ay_ratio_t = t;}
    double alpha_f = 0.0, alpha_r = 0.0;
    plant.slipAngles(x, applied_steer, alpha_f, alpha_r);
    peak_alpha_f = std::max(peak_alpha_f, std::abs(alpha_f));
    peak_alpha_r = std::max(peak_alpha_r, std::abs(alpha_r));
    const double beta = std::atan2(x(4), std::max(x(3), 0.1));
    peak_beta = std::max(peak_beta, std::abs(beta));
    if (std::abs(beta) > 20.0 * M_PI / 180.0) {
      if (!in_spin) {++spins; in_spin = true;}
    } else if (std::abs(beta) < 10.0 * M_PI / 180.0) {
      in_spin = false;
    }
    if (!std::isfinite(x(0)) || !std::isfinite(x(3))) {
      std::cerr << "plant diverged at t=" << t << "\n";
      break;
    }
  }

  // ---- Score each corner: brake onset and apex overspeed ------------------
  // For every apex the car passes, the worst pass is kept. Brake onset is the
  // upstream local speed maximum, so it is defined the same way for the car as
  // it was for the reference and the two are directly comparable.
  for (auto & c : corners) {
    for (size_t k = 1; k + 1 < log_s.size(); ++k) {
      // apex crossing: s passes apex_s going forward (ignore the wrap step)
      if (!(log_s[k - 1] < c.apex_s && log_s[k] >= c.apex_s)) {continue;}
      const double overspeed = log_v[k] - c.apex_v;
      if (c.car_v_at_apex >= 0.0 && overspeed <= c.car_v_at_apex - c.apex_v) {continue;}
      c.car_v_at_apex = log_v[k];
      double best_v = log_v[k];
      size_t best_k = k;
      for (size_t d = 1; d <= k && d <= 400; ++d) {
        const size_t j = k - d;
        if (log_s[j] > log_s[j + 1]) {break;}            // lap wrap
        if (c.apex_s - log_s[j] > 300.0) {break;}
        if (log_v[j] > best_v) {best_v = log_v[j]; best_k = j;}
        else if (log_v[j] < best_v - 0.5) {break;}
      }
      c.car_brake_s = log_s[best_k];
      c.car_entry_v = best_v;
      c.peak_alat = 0.0;
      c.peak_beta = 0.0;
      for (size_t j = best_k; j <= k; ++j) {
        c.peak_alat = std::max(c.peak_alat, log_alat[j]);
        c.peak_beta = std::max(c.peak_beta, log_beta[j]);
      }
    }
  }

  std::printf(
    "N=%d rate=%.0f alat=%.2f (eff %.2f) decel=%.2f accel=%.2f Q[r]=%.0f tau_cfg=%.2f mu=%.2f\n",
    o.N, o.control_rate, o.lateral_accel_max,
    std::min(o.lateral_accel_max, grip_state.budget(o)), o.decel_max, o.accel_max,
    o.q_r, o.tau_cfg, o.mu);
  std::printf(
    "  friction: schedule %s, plant mu %.3f -> %.3f | grip-longitudinal %s ellipse %s "
    "shape-util %s sigma-gain %.2f\n",
    o.mu_schedule.c_str(), muAt(o, 0.0), muAt(o, o.duration),
    o.grip_longitudinal ? "on" : "off", o.friction_ellipse ? "on" : "off",
    o.shape_util ? "on" : "off", o.sigma_gain);
  std::printf(
    "  estimators: %d coefficient sets (every %.0f s), %d fast publishes (%.1f Hz, %.1f s "
    "window, %.2f s lag)%s\n",
    sysid_updates, o.sysid_interval, fast_publishes, o.mu_fast_rate, o.mu_fast_window,
    o.mu_fast_lag,
    o.mu_fast_rate > 0.0 ? "" : " - NO fast channel, running on the identified D alone");
  std::printf(
    "  controller tire: f[%.3f %.3f %.3f %.3f] r[%.3f %.3f %.3f %.3f] | plant mu %.2f\n",
    o.ctrl_Bf, o.ctrl_Cf, o.ctrl_Df, o.ctrl_Ef, o.ctrl_Br, o.ctrl_Cr, o.ctrl_Dr, o.ctrl_Er, o.mu);
  std::printf(
    "  ref peak a_lat %.2f m/s^2 | curvature-clamped %zu/%zu | speed-clamped %zu\n",
    ref.maxLateralDemand(), ref.curvatureClampedCount(), ref.waypointCount(),
    ref.clampedWaypointCount());
  std::printf(
    "  max|e_y| %.2f m | peak a_lat %.2f m/s^2 | peak|beta| %.1f deg | spins %d | "
    "solver fail %d/%d | steer sign flips %d | distance %.0f m\n",
    max_ey, peak_alat, peak_beta * 180.0 / M_PI, spins, failures, cycles, sign_flips, distance);
  // The pass criteria. (1) is the whole point: the car must never ask the tires
  // for more than the surface can give, at any instant of the schedule.
  const bool pass_grip = worst_ay_ratio <= 1.0;
  const bool pass_solver = failures == 0;
  const bool pass_spin = spins == 0;
  std::printf(
    "  PASS/FAIL: peak a_lat / (mu_plant*g) = %.3f at t=%.1f s [%s] | solver failures %d [%s] | "
    "spins %d [%s] | peak|alpha| f %.1f deg r %.1f deg\n",
    worst_ay_ratio, worst_ay_ratio_t, pass_grip ? "PASS" : "FAIL", failures,
    pass_solver ? "PASS" : "FAIL", spins, pass_spin ? "PASS" : "FAIL",
    peak_alpha_f * 180.0 / M_PI, peak_alpha_r * 180.0 / M_PI);
  {
    std::vector<double> s = solve_all;
    std::sort(s.begin(), s.end());
    const double p95 = s.empty() ? 0.0 : s[static_cast<size_t>(0.95 * (s.size() - 1))];
    std::printf(
      "  horizon: adaptive %s gain %.2f s curv-gain %.1f m | dt mean %.4f min %.4f max %.4f s"
      " | preview mean %.2f s | RMS e_y %.3f m | solve mean %.2f p95 %.2f max %.2f ms\n",
      o.adaptive_distance ? "on" : "off", o.distance_gain, o.distance_curvature_gain,
      cycles > 0 ? dt_sum / cycles : 0.0, cycles > 0 ? dt_min_seen : 0.0, dt_max_seen,
      cycles > 0 ? o.N * dt_sum / cycles : 0.0,
      ey_n > 0 ? std::sqrt(ey_sq_sum / ey_n) : 0.0,
      cycles > 0 ? solve_sum / cycles : 0.0, p95, solve_max);
  }
  std::printf(
    "  tau: accel %.2f (%d samples) / decel %.2f (%d samples) vs plant %.2f / %.2f\n"
    "  tau peak %.2f / %.2f | rewrites %d / %d | cmd step mean %.2f max %.2f km/h\n",
    tau_cfg, tau_samples, tau_cfg_decel, tau_samples_decel, o.tau_plant,
    o.tau_plant_decel > 0.0 ? o.tau_plant_decel : o.tau_plant,
    tau_peak, tau_peak_decel, tau_rewrites, tau_rewrites_decel,
    ripple_n > 0 ? ripple_sum / ripple_n : 0.0, ripple_max);

  std::printf(
    "\n  corner  apex_s  v_ref  v_car  overspeed  brake_late_m  ref_brake_m  car_brake_m"
    "  a_lat  beta\n");
  int scored = 0, late = 0;
  double sum_over = 0.0, worst_over = -1e9, sum_late = 0.0, worst_late = -1e9;
  for (size_t i = 0; i < corners.size(); ++i) {
    const Corner & c = corners[i];
    if (c.car_v_at_apex < 0.0) {continue;}
    const double over = c.car_v_at_apex - c.apex_v;
    const double ref_brake_m = c.apex_s - c.ref_brake_s;
    const double car_brake_m = c.apex_s - c.car_brake_s;
    const double brake_late = ref_brake_m - car_brake_m;   // >0 = car braked later
    ++scored;
    if (brake_late > 5.0) {++late;}
    sum_over += over; worst_over = std::max(worst_over, over);
    sum_late += brake_late; worst_late = std::max(worst_late, brake_late);
    std::printf(
      "  %5zu  %6.0f  %5.2f  %5.2f  %9.2f  %12.1f  %11.1f  %11.1f  %5.2f  %4.1f\n",
      i, c.apex_s, c.apex_v, c.car_v_at_apex, over, brake_late, ref_brake_m, car_brake_m,
      c.peak_alat, c.peak_beta * 180.0 / M_PI);
  }
  if (scored > 0) {
    std::printf(
      "  %d/%zu corners scored | overspeed mean %.2f worst %.2f m/s | brake onset late "
      "mean %.1f worst %.1f m | %d corners braking >5 m late\n",
      scored, corners.size(), sum_over / scored, worst_over, sum_late / scored, worst_late, late);
  }
  (void)track_len;
  return 0;
}
