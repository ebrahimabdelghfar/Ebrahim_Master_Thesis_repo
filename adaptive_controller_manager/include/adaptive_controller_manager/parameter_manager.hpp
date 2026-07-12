#ifndef ADAPTIVE_CONTROLLER_MANAGER__PARAMETER_MANAGER_HPP_
#define ADAPTIVE_CONTROLLER_MANAGER__PARAMETER_MANAGER_HPP_

#include <array>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace adaptive_controller_manager
{

struct TopicsConfig
{
  std::string odom_topic;
  std::string waypoint_topic;
  std::string pp_drive_topic;
  std::string mpc_drive_topic;
  std::string pp_state_topic;
  std::string pp_health_topic;
  std::string mpc_status_topic;
  std::string sysid_first_run_topic;
  std::string drive_topic;
  std::string start_working_pp_topic;
  std::string start_working_mpc_topic;
  std::string sysid_update_params_service;
  std::string mpc_update_params_service;
};

// Optional, best-effort forward of every accepted sysid/update_params
// submission to a passive benchmarking node (tire_force_benchmark), so it
// can evaluate the freshly-identified model instead of a hardcoded one.
// Disabled by default - opt in per-deployment via config.
struct BenchmarkForwardConfig
{
  bool enable{false};
  std::string service{"benchmark/update_params"};
};

struct SafetyConfig
{
  double control_rate_hz{20.0};
  double v_min{0.5};
  double e_y_max{0.1};
  double theta_max{0.1};
  double delta_t_state_max{0.1};
  int error_convergence_window{20};
  double delta_t_timeout{1.0};
  double delta_t_switch{1.0};

  // Safety 7: block the PP->MPC handover if a sharp corner is imminent -
  // starting the switch ramp (steering jump + speed blend) right before a
  // tight corner risks destabilizing the vehicle mid-turn.
  double corner_lookahead_distance{3.0};   // m, ahead of the nearest waypoint
  double kappa_max_for_switch{0.35};       // rad/m, above this counts as sharp

  // Global cap on how fast a commanded speed may DECREASE tick-to-tick
  // (m/s^2) - covers the switch ramp, EMERGENCY_HALT's zero command, and
  // the stale-data fallback, so none of them ever reads as an instant
  // stop. Matches f1tenth_simulator/mpc_path_tracking's existing
  // max_decel/limits.decel_max physical limit.
  double max_decel_mps2{8.26};
};

// Plausible-range bounds for a submitted tire param set, fixed order
// [Bf, Cf, Df, Ef, Br, Cr, Dr, Er] (matches IdentifiedParam.srv and
// mpc_path_tracking's tire.* config block). These are coarse sanity
// bounds meant to catch a grossly wrong identification (NaN, sign flip,
// orders-of-magnitude error), not a validated physical range for any
// specific vehicle - retune per-vehicle via config.
struct TireBounds
{
  std::array<double, 8> min{{0.0, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, -10.0}};
  std::array<double, 8> max{{50.0, 10.0, 3.0, 10.0, 50.0, 10.0, 3.0, 10.0}};
};

// Declares and reads every ROS parameter for adaptive_controller_manager_node
// (topics, safety-gate thresholds, tire-param plausibility bounds),
// matching config/adaptive_controller_manager.yaml, mirroring
// mpc_path_tracking's ParameterManager pattern.
class ParameterManager
{
public:
  explicit ParameterManager(rclcpp::Node * node);

  void declareAll();

  TopicsConfig topics() const;
  SafetyConfig safety() const;
  TireBounds tireBounds() const;
  BenchmarkForwardConfig benchmarkForward() const;

private:
  rclcpp::Node * node_;
};

}  // namespace adaptive_controller_manager

#endif  // ADAPTIVE_CONTROLLER_MANAGER__PARAMETER_MANAGER_HPP_
