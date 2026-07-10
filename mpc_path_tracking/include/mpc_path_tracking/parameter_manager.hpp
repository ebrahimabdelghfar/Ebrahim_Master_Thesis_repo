#ifndef MPC_PATH_TRACKING__PARAMETER_MANAGER_HPP_
#define MPC_PATH_TRACKING__PARAMETER_MANAGER_HPP_

#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mpc_path_tracking/mpc_controller.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

namespace mpc_path_tracking
{

struct TopicsConfig
{
  std::string odom_topic;
  std::string waypoint_topic;
  std::string drive_topic;
  std::string predicted_trajectory_topic;
  std::string status_topic;
  std::string debug_topic_prefix;
};

struct SolverConfig
{
  std::string backend;
  int max_iter{200};
  double eps_abs{1e-4};
  double eps_rel{1e-4};
  bool warm_start{true};
  bool polish{true};
  double time_limit_ms{15.0};
  std::string fallback_on_failure{"hold_last"};
};

struct DebugConfig
{
  bool enabled{true};
  bool publish_predicted_path{true};
  bool publish_cost_breakdown{true};
  bool log_solve_time{true};
};

// Declares and validates every ROS parameter for the node (topics,
// horizon/timing, cost matrices, vehicle/tire params, actuator limits,
// solver settings, debug options), matching the YAML schema in
// config/mpc_path_tracking.yaml. Also wires a live-tune callback for the
// subset of parameters safe to change at runtime (cost weights, limits),
// mirroring pure_pursuit_node.py's add_on_set_parameters_callback pattern.
class ParameterManager
{
public:
  explicit ParameterManager(rclcpp::Node * node);

  void declareAll();

  TopicsConfig topics() const;
  MpcConfig mpcConfig() const;
  VehicleParams vehicleParams() const;
  TireParams tireParams() const;
  SolverConfig solverConfig() const;
  DebugConfig debugConfig() const;
  double controlRateHz() const;
  bool dtAdaptive() const;

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  static Eigen::Matrix<double, 5, 1> toVec5(const std::vector<double> & v, const char * name);
  static Eigen::Vector2d toVec2(const std::vector<double> & v, const char * name);

  rclcpp::Node * node_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__PARAMETER_MANAGER_HPP_
