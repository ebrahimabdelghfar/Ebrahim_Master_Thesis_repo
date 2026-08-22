#ifndef MPC_PATH_TRACKING__DEBUG_PUBLISHER_HPP_
#define MPC_PATH_TRACKING__DEBUG_PUBLISHER_HPP_

#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "mpc_path_tracking/parameter_manager.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

namespace mpc_path_tracking
{

// Publishes the predicted-trajectory, controller-status and scalar debug
// topics, gated by DebugConfig so a production run can disable the
// per-cycle Path/Float64 publishing overhead entirely.
class DebugPublisher
{
public:
  DebugPublisher(rclcpp::Node * node, const TopicsConfig & topics, const DebugConfig & debug_cfg);

  void publishPredictedPath(
    const std::vector<State> & states, const std::string & frame_id, const rclcpp::Time & stamp);

  // Extra per-cycle facts that are otherwise invisible from outside the node.
  // In particular the EFFECTIVE tire params: mpc/update_params calls
  // MpcController::setTireParams directly and never writes back to the ROS
  // parameters, so `ros2 param get .. tire.Df` keeps reporting the yaml value
  // no matter what sysid pushed. This is the only truthful readout.
  struct StatusInfo
  {
    TireParams tire;
    int infeasible_ref_stages{0};
    bool relinearized{false};
    double x0_prediction_s{0.0};   // how far x0 was rolled forward for latency
  };

  void publishStatus(
    bool solved, double solve_time_ms, double cost, const std::string & message,
    const rclcpp::Time & stamp);

  void publishStatus(
    bool solved, double solve_time_ms, double cost, const std::string & message,
    const StatusInfo & info, const rclcpp::Time & stamp);

  void publishDebugScalars(double e_y, double e_psi, double cost, double solve_time_ms);

private:
  rclcpp::Node * node_;
  DebugConfig debug_cfg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr e_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr e_psi_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cost_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_time_pub_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__DEBUG_PUBLISHER_HPP_
