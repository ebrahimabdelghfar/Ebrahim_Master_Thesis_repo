#include "adaptive_controller_manager/parameter_manager.hpp"

#include <vector>

namespace adaptive_controller_manager
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
  node_->declare_parameter<std::string>("pp_drive_topic", "pp/drive_cmd");
  node_->declare_parameter<std::string>("mpc_drive_topic", "mpc/drive_cmd");
  node_->declare_parameter<std::string>("pp_state_topic", "pp_state");
  node_->declare_parameter<std::string>("pp_health_topic", "pp_health");
  node_->declare_parameter<std::string>("mpc_status_topic", "/mpc/status");
  node_->declare_parameter<std::string>("sysid_first_run_topic", "sysid/first_run");
  node_->declare_parameter<std::string>("drive_topic", "/drive");
  node_->declare_parameter<std::string>("start_working_pp_topic", "Start_Working_pp");
  node_->declare_parameter<std::string>("start_working_mpc_topic", "Start_Working_mpc");
  node_->declare_parameter<std::string>("sysid_update_params_service", "sysid/update_params");
  node_->declare_parameter<std::string>("mpc_update_params_service", "mpc/update_params");

  // Safety-gate thresholds
  node_->declare_parameter<double>("control_rate_hz", 20.0);
  node_->declare_parameter<double>("v_min", 0.5);
  node_->declare_parameter<double>("e_y_max", 0.1);
  node_->declare_parameter<double>("theta_max", 0.1);
  node_->declare_parameter<double>("delta_t_state_max", 0.1);
  node_->declare_parameter<int>("error_convergence_window", 20);
  node_->declare_parameter<double>("delta_t_timeout", 1.0);
  node_->declare_parameter<double>("delta_t_switch", 1.0);
  node_->declare_parameter<double>("corner_lookahead_distance", 3.0);
  node_->declare_parameter<double>("kappa_max_for_switch", 0.35);
  node_->declare_parameter<double>("max_decel_mps2", 8.26);
  node_->declare_parameter<double>("mpc_forward_retry_period_s", 5.0);

  // Tire-param plausibility bounds, fixed order [Bf,Cf,Df,Ef,Br,Cr,Dr,Er]
  node_->declare_parameter<std::vector<double>>(
    "tire_param_min", {0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, -10.0});
  node_->declare_parameter<std::vector<double>>(
    "tire_param_max", {50.0, 10.0, 3.0, 10.0, 50.0, 10.0, 3.0, 10.0});

  // Benchmark forwarding (optional, disabled by default)
  node_->declare_parameter<bool>("benchmark_update_params_enable", false);
  node_->declare_parameter<std::string>(
    "benchmark_update_params_service", "benchmark/update_params");
}

TopicsConfig ParameterManager::topics() const
{
  TopicsConfig t;
  t.odom_topic = node_->get_parameter("odom_topic").as_string();
  t.waypoint_topic = node_->get_parameter("waypoint_topic").as_string();
  t.pp_drive_topic = node_->get_parameter("pp_drive_topic").as_string();
  t.mpc_drive_topic = node_->get_parameter("mpc_drive_topic").as_string();
  t.pp_state_topic = node_->get_parameter("pp_state_topic").as_string();
  t.pp_health_topic = node_->get_parameter("pp_health_topic").as_string();
  t.mpc_status_topic = node_->get_parameter("mpc_status_topic").as_string();
  t.sysid_first_run_topic = node_->get_parameter("sysid_first_run_topic").as_string();
  t.drive_topic = node_->get_parameter("drive_topic").as_string();
  t.start_working_pp_topic = node_->get_parameter("start_working_pp_topic").as_string();
  t.start_working_mpc_topic = node_->get_parameter("start_working_mpc_topic").as_string();
  t.sysid_update_params_service = node_->get_parameter("sysid_update_params_service").as_string();
  t.mpc_update_params_service = node_->get_parameter("mpc_update_params_service").as_string();
  return t;
}

SafetyConfig ParameterManager::safety() const
{
  SafetyConfig s;
  s.control_rate_hz = node_->get_parameter("control_rate_hz").as_double();
  s.v_min = node_->get_parameter("v_min").as_double();
  s.e_y_max = node_->get_parameter("e_y_max").as_double();
  s.theta_max = node_->get_parameter("theta_max").as_double();
  s.delta_t_state_max = node_->get_parameter("delta_t_state_max").as_double();
  s.error_convergence_window =
    static_cast<int>(node_->get_parameter("error_convergence_window").as_int());
  s.delta_t_timeout = node_->get_parameter("delta_t_timeout").as_double();
  s.delta_t_switch = node_->get_parameter("delta_t_switch").as_double();
  s.corner_lookahead_distance = node_->get_parameter("corner_lookahead_distance").as_double();
  s.kappa_max_for_switch = node_->get_parameter("kappa_max_for_switch").as_double();
  s.max_decel_mps2 = node_->get_parameter("max_decel_mps2").as_double();
  s.mpc_forward_retry_period_s =
    node_->get_parameter("mpc_forward_retry_period_s").as_double();
  return s;
}

TireBounds ParameterManager::tireBounds() const
{
  TireBounds b;
  const auto min_v = node_->get_parameter("tire_param_min").as_double_array();
  const auto max_v = node_->get_parameter("tire_param_max").as_double_array();
  if (min_v.size() == 8 && max_v.size() == 8) {
    for (size_t i = 0; i < 8; ++i) {
      b.min[i] = min_v[i];
      b.max[i] = max_v[i];
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "tire_param_min/tire_param_max must each have exactly 8 elements - using built-in defaults");
  }
  return b;
}

BenchmarkForwardConfig ParameterManager::benchmarkForward() const
{
  BenchmarkForwardConfig b;
  b.enable = node_->get_parameter("benchmark_update_params_enable").as_bool();
  b.service = node_->get_parameter("benchmark_update_params_service").as_string();
  return b;
}

}  // namespace adaptive_controller_manager
