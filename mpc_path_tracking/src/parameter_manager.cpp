#include "mpc_path_tracking/parameter_manager.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <stdexcept>

namespace mpc_path_tracking
{

ParameterManager::ParameterManager(rclcpp::Node * node)
: node_(node)
{
}

void ParameterManager::declareAll()
{
  // Topics
  node_->declare_parameter<std::string>("odom_topic", "/odom");
  node_->declare_parameter<std::string>("waypoint_topic", "/raceline_waypoints");
  node_->declare_parameter<std::string>("drive_topic", "mpc/drive_cmd");
  node_->declare_parameter<std::string>("predicted_trajectory_topic", "/mpc/predicted_path");
  node_->declare_parameter<std::string>("status_topic", "/mpc/status");
  node_->declare_parameter<std::string>("debug_topic_prefix", "/mpc/debug");
  node_->declare_parameter<std::string>("enable_topic", "Start_Working_mpc");
  node_->declare_parameter<std::string>("param_service", "mpc/update_params");

  // Managed-mode gating (see adaptive_controller_manager). true: always
  // publish, ignore Start_Working_mpc (solo tuning against the simulator).
  node_->declare_parameter<bool>("standalone_mode", false);

  // Horizon / timing
  node_->declare_parameter<int>("horizon.N", 20);
  node_->declare_parameter<double>("horizon.dt_min", 0.02);
  node_->declare_parameter<double>("horizon.dt_max", 0.08);
  node_->declare_parameter<double>("horizon.horizon_distance_m", 8.0);
  node_->declare_parameter<double>("horizon.control_rate_hz", 20.0);
  node_->declare_parameter<bool>("horizon.dt_adaptive", true);

  // Cost matrices (diagonal weights)
  node_->declare_parameter<std::vector<double>>("cost.Q", {50.0, 30.0, 10.0, 1.0, 1.0});
  node_->declare_parameter<std::vector<double>>("cost.R", {5.0, 1.0});
  node_->declare_parameter<std::vector<double>>("cost.R_rate", {10.0, 2.0});
  node_->declare_parameter<std::vector<double>>("cost.Qf", {80.0, 40.0, 15.0, 1.0, 1.0});

  // Vehicle geometry / mass (defaults from On-Track-SysID/models/SIM/SIM_pacejka.txt)
  node_->declare_parameter<double>("vehicle.mass", 3.54);
  node_->declare_parameter<double>("vehicle.Iz", 0.05797);
  node_->declare_parameter<double>("vehicle.l_f", 0.162);
  node_->declare_parameter<double>("vehicle.l_r", 0.145);
  node_->declare_parameter<double>("vehicle.h_cg", 0.02);

  // Pacejka tire model (defaults from On-Track-SysID/models/SIM/SIM_pacejka.txt)
  node_->declare_parameter<double>("tire.Bf", 2.4128);
  node_->declare_parameter<double>("tire.Cf", 4.8155);
  node_->declare_parameter<double>("tire.Df", 0.5922);
  node_->declare_parameter<double>("tire.Ef", 5.0);
  node_->declare_parameter<double>("tire.Br", 14.4445);
  node_->declare_parameter<double>("tire.Cr", 1.2129);
  node_->declare_parameter<double>("tire.Dr", 0.6842);
  node_->declare_parameter<double>("tire.Er", 0.8526);

  // Actuator limits / rate limits (defaults from f1tenth_simulator/params.yaml)
  node_->declare_parameter<double>("limits.steering_angle_max", 0.4189);
  node_->declare_parameter<double>("limits.steering_angle_min", -0.4189);
  node_->declare_parameter<double>("limits.steering_rate_max", 3.2);
  node_->declare_parameter<double>("limits.speed_max", 7.0);
  node_->declare_parameter<double>("limits.speed_min", 0.0);
  node_->declare_parameter<double>("limits.accel_max", 7.51);
  node_->declare_parameter<double>("limits.decel_max", 8.26);
  node_->declare_parameter<double>("limits.jerk_max", 1000.0);

  // Solver settings
  node_->declare_parameter<std::string>("solver.backend", "osqp");
  node_->declare_parameter<int>("solver.max_iter", 200);
  node_->declare_parameter<double>("solver.eps_abs", 1.0e-4);
  node_->declare_parameter<double>("solver.eps_rel", 1.0e-4);
  node_->declare_parameter<bool>("solver.warm_start", true);
  node_->declare_parameter<bool>("solver.polish", true);
  node_->declare_parameter<double>("solver.time_limit_ms", 15.0);
  node_->declare_parameter<std::string>("solver.fallback_on_failure", "hold_last");

  // Debug / diagnostics
  node_->declare_parameter<bool>("debug.enabled", true);
  node_->declare_parameter<bool>("debug.publish_predicted_path", true);
  node_->declare_parameter<bool>("debug.publish_cost_breakdown", true);
  node_->declare_parameter<bool>("debug.log_solve_time", true);
  node_->declare_parameter<std::string>("debug.log_level", "info");

  const std::string log_level = node_->get_parameter("debug.log_level").as_string();
  if (log_level == "debug") {
    node_->get_logger().set_level(rclcpp::Logger::Level::Debug);
  } else if (log_level == "warn") {
    node_->get_logger().set_level(rclcpp::Logger::Level::Warn);
  } else if (log_level == "error") {
    node_->get_logger().set_level(rclcpp::Logger::Level::Error);
  } else {
    node_->get_logger().set_level(rclcpp::Logger::Level::Info);
  }

  callback_handle_ = node_->add_on_set_parameters_callback(
    std::bind(&ParameterManager::onSetParameters, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
// printAll(): dumps every declared parameter to the logger at INFO level,
//             grouped by section, for quick startup verification.
// ---------------------------------------------------------------------------
void ParameterManager::printAll() const
{
  auto log = node_->get_logger();

  RCLCPP_INFO(log, "╔══════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(log, "║          MPC PATH TRACKING — LOADED PARAMETERS              ║");
  RCLCPP_INFO(log, "╚══════════════════════════════════════════════════════════════╝");

  // --- Topics ---
  RCLCPP_INFO(log, "──── Topics ────");
  RCLCPP_INFO(log, "  odom_topic                : %s", node_->get_parameter("odom_topic").as_string().c_str());
  RCLCPP_INFO(log, "  waypoint_topic            : %s", node_->get_parameter("waypoint_topic").as_string().c_str());
  RCLCPP_INFO(log, "  drive_topic               : %s", node_->get_parameter("drive_topic").as_string().c_str());
  RCLCPP_INFO(log, "  predicted_trajectory_topic : %s", node_->get_parameter("predicted_trajectory_topic").as_string().c_str());
  RCLCPP_INFO(log, "  status_topic              : %s", node_->get_parameter("status_topic").as_string().c_str());
  RCLCPP_INFO(log, "  debug_topic_prefix        : %s", node_->get_parameter("debug_topic_prefix").as_string().c_str());
  RCLCPP_INFO(log, "  enable_topic              : %s", node_->get_parameter("enable_topic").as_string().c_str());
  RCLCPP_INFO(log, "  param_service             : %s", node_->get_parameter("param_service").as_string().c_str());

  // --- Mode ---
  RCLCPP_INFO(log, "──── Mode ────");
  RCLCPP_INFO(log, "  standalone_mode           : %s", node_->get_parameter("standalone_mode").as_bool() ? "true" : "false");

  // --- Horizon / Timing ---
  RCLCPP_INFO(log, "──── Horizon / Timing ────");
  RCLCPP_INFO(log, "  N                         : %ld", node_->get_parameter("horizon.N").as_int());
  RCLCPP_INFO(log, "  dt_min                    : %.4f s", node_->get_parameter("horizon.dt_min").as_double());
  RCLCPP_INFO(log, "  dt_max                    : %.4f s", node_->get_parameter("horizon.dt_max").as_double());
  RCLCPP_INFO(log, "  dt_adaptive               : %s", node_->get_parameter("horizon.dt_adaptive").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  horizon_distance_m        : %.2f m", node_->get_parameter("horizon.horizon_distance_m").as_double());
  RCLCPP_INFO(log, "  control_rate_hz           : %.1f Hz", node_->get_parameter("horizon.control_rate_hz").as_double());

  // --- Cost Matrices ---
  auto fmtVec = [](const std::vector<double> & v) -> std::string {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); ++i) {
      if (i > 0) s += ", ";
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.4f", v[i]);
      s += buf;
    }
    s += "]";
    return s;
  };

  RCLCPP_INFO(log, "──── Cost Matrices ────");
  RCLCPP_INFO(log, "  Q      [e_y, e_psi, vx_err, vy, r_err] : %s",
    fmtVec(node_->get_parameter("cost.Q").as_double_array()).c_str());
  RCLCPP_INFO(log, "  R      [steering, accel]                : %s",
    fmtVec(node_->get_parameter("cost.R").as_double_array()).c_str());
  RCLCPP_INFO(log, "  R_rate [steering, accel]                : %s",
    fmtVec(node_->get_parameter("cost.R_rate").as_double_array()).c_str());
  RCLCPP_INFO(log, "  Qf     [e_y, e_psi, vx_err, vy, r_err] : %s",
    fmtVec(node_->get_parameter("cost.Qf").as_double_array()).c_str());

  // --- Vehicle ---
  RCLCPP_INFO(log, "──── Vehicle Geometry / Mass ────");
  RCLCPP_INFO(log, "  mass                      : %.4f kg", node_->get_parameter("vehicle.mass").as_double());
  RCLCPP_INFO(log, "  Iz                        : %.5f kg·m²", node_->get_parameter("vehicle.Iz").as_double());
  RCLCPP_INFO(log, "  l_f                       : %.4f m", node_->get_parameter("vehicle.l_f").as_double());
  RCLCPP_INFO(log, "  l_r                       : %.4f m", node_->get_parameter("vehicle.l_r").as_double());
  RCLCPP_INFO(log, "  h_cg                      : %.4f m", node_->get_parameter("vehicle.h_cg").as_double());

  // --- Tire (Pacejka) ---
  RCLCPP_INFO(log, "──── Pacejka Tire Model ────");
  RCLCPP_INFO(log, "  Front: Bf=%.4f  Cf=%.4f  Df=%.4f  Ef=%.4f",
    node_->get_parameter("tire.Bf").as_double(), node_->get_parameter("tire.Cf").as_double(),
    node_->get_parameter("tire.Df").as_double(), node_->get_parameter("tire.Ef").as_double());
  RCLCPP_INFO(log, "  Rear:  Br=%.4f  Cr=%.4f  Dr=%.4f  Er=%.4f",
    node_->get_parameter("tire.Br").as_double(), node_->get_parameter("tire.Cr").as_double(),
    node_->get_parameter("tire.Dr").as_double(), node_->get_parameter("tire.Er").as_double());

  // --- Actuator Limits ---
  RCLCPP_INFO(log, "──── Actuator Limits ────");
  RCLCPP_INFO(log, "  steering_angle_max        : %.4f rad", node_->get_parameter("limits.steering_angle_max").as_double());
  RCLCPP_INFO(log, "  steering_angle_min        : %.4f rad", node_->get_parameter("limits.steering_angle_min").as_double());
  RCLCPP_INFO(log, "  steering_rate_max         : %.2f rad/s", node_->get_parameter("limits.steering_rate_max").as_double());
  RCLCPP_INFO(log, "  speed_max                 : %.2f m/s", node_->get_parameter("limits.speed_max").as_double());
  RCLCPP_INFO(log, "  speed_min                 : %.2f m/s", node_->get_parameter("limits.speed_min").as_double());
  RCLCPP_INFO(log, "  accel_max                 : %.2f m/s²", node_->get_parameter("limits.accel_max").as_double());
  RCLCPP_INFO(log, "  decel_max                 : %.2f m/s²", node_->get_parameter("limits.decel_max").as_double());
  RCLCPP_INFO(log, "  jerk_max                  : %.2f m/s³", node_->get_parameter("limits.jerk_max").as_double());

  // --- Solver ---
  RCLCPP_INFO(log, "──── Solver Settings ────");
  RCLCPP_INFO(log, "  backend                   : %s", node_->get_parameter("solver.backend").as_string().c_str());
  RCLCPP_INFO(log, "  max_iter                  : %ld", node_->get_parameter("solver.max_iter").as_int());
  RCLCPP_INFO(log, "  eps_abs                   : %.1e", node_->get_parameter("solver.eps_abs").as_double());
  RCLCPP_INFO(log, "  eps_rel                   : %.1e", node_->get_parameter("solver.eps_rel").as_double());
  RCLCPP_INFO(log, "  warm_start                : %s", node_->get_parameter("solver.warm_start").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  polish                    : %s", node_->get_parameter("solver.polish").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  time_limit_ms             : %.1f ms", node_->get_parameter("solver.time_limit_ms").as_double());
  RCLCPP_INFO(log, "  fallback_on_failure       : %s", node_->get_parameter("solver.fallback_on_failure").as_string().c_str());

  // --- Debug ---
  RCLCPP_INFO(log, "──── Debug / Diagnostics ────");
  RCLCPP_INFO(log, "  enabled                   : %s", node_->get_parameter("debug.enabled").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  publish_predicted_path    : %s", node_->get_parameter("debug.publish_predicted_path").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  publish_cost_breakdown    : %s", node_->get_parameter("debug.publish_cost_breakdown").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  log_solve_time            : %s", node_->get_parameter("debug.log_solve_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(log, "  log_level                 : %s", node_->get_parameter("debug.log_level").as_string().c_str());

  RCLCPP_INFO(log, "══════════════════════════════════════════════════════════════");
}

Eigen::Matrix<double, 5, 1> ParameterManager::toVec5(const std::vector<double> & v, const char * name)
{
  if (v.size() != 5) {
    throw std::runtime_error(std::string("parameter '") + name + "' must have exactly 5 elements");
  }
  Eigen::Matrix<double, 5, 1> out;
  for (int i = 0; i < 5; ++i) {out(i) = v[i];}
  return out;
}

Eigen::Vector2d ParameterManager::toVec2(const std::vector<double> & v, const char * name)
{
  if (v.size() != 2) {
    throw std::runtime_error(std::string("parameter '") + name + "' must have exactly 2 elements");
  }
  return Eigen::Vector2d(v[0], v[1]);
}

TopicsConfig ParameterManager::topics() const
{
  TopicsConfig t;
  t.odom_topic = node_->get_parameter("odom_topic").as_string();
  t.waypoint_topic = node_->get_parameter("waypoint_topic").as_string();
  t.drive_topic = node_->get_parameter("drive_topic").as_string();
  t.predicted_trajectory_topic = node_->get_parameter("predicted_trajectory_topic").as_string();
  t.status_topic = node_->get_parameter("status_topic").as_string();
  t.debug_topic_prefix = node_->get_parameter("debug_topic_prefix").as_string();
  t.enable_topic = node_->get_parameter("enable_topic").as_string();
  t.param_service = node_->get_parameter("param_service").as_string();
  return t;
}

MpcConfig ParameterManager::mpcConfig() const
{
  MpcConfig c;
  c.N = static_cast<int>(node_->get_parameter("horizon.N").as_int());
  c.dt_min = node_->get_parameter("horizon.dt_min").as_double();
  c.dt_max = node_->get_parameter("horizon.dt_max").as_double();
  c.horizon_distance_m = node_->get_parameter("horizon.horizon_distance_m").as_double();

  c.cost.Q = toVec5(node_->get_parameter("cost.Q").as_double_array(), "cost.Q");
  c.cost.Qf = toVec5(node_->get_parameter("cost.Qf").as_double_array(), "cost.Qf");
  c.cost.R = toVec2(node_->get_parameter("cost.R").as_double_array(), "cost.R");
  c.cost.Rrate = toVec2(node_->get_parameter("cost.R_rate").as_double_array(), "cost.R_rate");

  c.limits.steering_min = node_->get_parameter("limits.steering_angle_min").as_double();
  c.limits.steering_max = node_->get_parameter("limits.steering_angle_max").as_double();
  c.limits.steering_rate_max = node_->get_parameter("limits.steering_rate_max").as_double();
  c.limits.accel_max = node_->get_parameter("limits.accel_max").as_double();
  c.limits.accel_min = -node_->get_parameter("limits.decel_max").as_double();
  c.limits.jerk_max = node_->get_parameter("limits.jerk_max").as_double();
  return c;
}

VehicleParams ParameterManager::vehicleParams() const
{
  VehicleParams v;
  v.mass = node_->get_parameter("vehicle.mass").as_double();
  v.Iz = node_->get_parameter("vehicle.Iz").as_double();
  v.l_f = node_->get_parameter("vehicle.l_f").as_double();
  v.l_r = node_->get_parameter("vehicle.l_r").as_double();
  v.h_cg = node_->get_parameter("vehicle.h_cg").as_double();
  return v;
}

TireParams ParameterManager::tireParams() const
{
  TireParams t;
  t.Bf = node_->get_parameter("tire.Bf").as_double();
  t.Cf = node_->get_parameter("tire.Cf").as_double();
  t.Df = node_->get_parameter("tire.Df").as_double();
  t.Ef = node_->get_parameter("tire.Ef").as_double();
  t.Br = node_->get_parameter("tire.Br").as_double();
  t.Cr = node_->get_parameter("tire.Cr").as_double();
  t.Dr = node_->get_parameter("tire.Dr").as_double();
  t.Er = node_->get_parameter("tire.Er").as_double();
  return t;
}

SolverConfig ParameterManager::solverConfig() const
{
  SolverConfig s;
  s.backend = node_->get_parameter("solver.backend").as_string();
  s.max_iter = static_cast<int>(node_->get_parameter("solver.max_iter").as_int());
  s.eps_abs = node_->get_parameter("solver.eps_abs").as_double();
  s.eps_rel = node_->get_parameter("solver.eps_rel").as_double();
  s.warm_start = node_->get_parameter("solver.warm_start").as_bool();
  s.polish = node_->get_parameter("solver.polish").as_bool();
  s.time_limit_ms = node_->get_parameter("solver.time_limit_ms").as_double();
  s.fallback_on_failure = node_->get_parameter("solver.fallback_on_failure").as_string();
  return s;
}

DebugConfig ParameterManager::debugConfig() const
{
  DebugConfig d;
  d.enabled = node_->get_parameter("debug.enabled").as_bool();
  d.publish_predicted_path = node_->get_parameter("debug.publish_predicted_path").as_bool();
  d.publish_cost_breakdown = node_->get_parameter("debug.publish_cost_breakdown").as_bool();
  d.log_solve_time = node_->get_parameter("debug.log_solve_time").as_bool();
  return d;
}

double ParameterManager::controlRateHz() const
{
  return node_->get_parameter("horizon.control_rate_hz").as_double();
}

bool ParameterManager::dtAdaptive() const
{
  return node_->get_parameter("horizon.dt_adaptive").as_bool();
}

bool ParameterManager::standaloneMode() const
{
  return node_->get_parameter("standalone_mode").as_bool();
}

rcl_interfaces::msg::SetParametersResult ParameterManager::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  static const std::vector<std::string> kArrayParams = {"cost.Q", "cost.Qf", "cost.R", "cost.R_rate"};

  for (const auto & p : params) {
    const auto & name = p.get_name();
    const bool is_array_param =
      std::find(kArrayParams.begin(), kArrayParams.end(), name) != kArrayParams.end();
    if (is_array_param) {
      const auto & arr = p.as_double_array();
      const size_t expected = (name == "cost.Q" || name == "cost.Qf") ? 5 : 2;
      if (arr.size() != expected) {
        result.successful = false;
        result.reason = name + " must have " + std::to_string(expected) + " elements";
        return result;
      }
    }
  }
  return result;
}

}  // namespace mpc_path_tracking
