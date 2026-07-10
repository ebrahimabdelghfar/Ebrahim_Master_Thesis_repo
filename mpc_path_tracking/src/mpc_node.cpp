#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "f1tenth_msgs/msg/waypoint_array.hpp"
#include "mpc_path_tracking/debug_publisher.hpp"
#include "mpc_path_tracking/mpc_controller.hpp"
#include "mpc_path_tracking/parameter_manager.hpp"
#include "mpc_path_tracking/reference_trajectory_handler.hpp"
#include "mpc_path_tracking/solver_interface.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

using std::placeholders::_1;

namespace mpc_path_tracking
{

class MpcNode : public rclcpp::Node
{
public:
  MpcNode()
  : rclcpp::Node("mpc_path_tracking"), param_manager_(this)
  {
    param_manager_.declareAll();
    topics_ = param_manager_.topics();
    solver_cfg_ = param_manager_.solverConfig();

    const VehicleParams vehicle_params = param_manager_.vehicleParams();
    const TireParams tire_params = param_manager_.tireParams();
    vehicle_model_ = std::make_unique<VehicleModel>(vehicle_params, tire_params);

    OsqpSolverSettings osqp_settings;
    osqp_settings.max_iter = solver_cfg_.max_iter;
    osqp_settings.eps_abs = solver_cfg_.eps_abs;
    osqp_settings.eps_rel = solver_cfg_.eps_rel;
    osqp_settings.warm_start = solver_cfg_.warm_start;
    osqp_settings.polish = solver_cfg_.polish;
    osqp_settings.time_limit_s = solver_cfg_.time_limit_ms / 1000.0;
    auto solver = std::make_unique<OsqpMpcSolver>(osqp_settings);

    controller_ = std::make_unique<MpcController>(
      *vehicle_model_, std::move(solver), param_manager_.mpcConfig());

    debug_pub_ = std::make_unique<DebugPublisher>(this, topics_, param_manager_.debugConfig());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topics_.odom_topic, rclcpp::QoS(1), std::bind(&MpcNode::odomCallback, this, _1));

    rclcpp::QoS waypoint_qos(1);
    waypoint_qos.transient_local();
    waypoint_sub_ = create_subscription<f1tenth_msgs::msg::WaypointArray>(
      topics_.waypoint_topic, waypoint_qos, std::bind(&MpcNode::waypointCallback, this, _1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      topics_.drive_topic, rclcpp::QoS(1));

    const double rate_hz = std::max(param_manager_.controlRateHz(), 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&MpcNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(), "mpc_path_tracking ready: odom=%s waypoints=%s drive=%s",
      topics_.odom_topic.c_str(), topics_.waypoint_topic.c_str(), topics_.drive_topic.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_state_(0) = msg->pose.pose.position.x;
    current_state_(1) = msg->pose.pose.position.y;
    current_state_(2) = tf2::getYaw(msg->pose.pose.orientation);
    current_state_(3) = msg->twist.twist.linear.x;
    current_state_(4) = msg->twist.twist.linear.y;
    current_state_(5) = msg->twist.twist.angular.z;
    last_odom_stamp_ = now();
    has_odom_ = true;
  }

  void waypointCallback(const f1tenth_msgs::msg::WaypointArray::SharedPtr msg)
  {
    ref_handler_.setWaypoints(*msg);
    RCLCPP_INFO(get_logger(), "received %zu raceline waypoints", ref_handler_.waypointCount());
  }

  void applyFallback(const std::string & reason)
  {
    const std::string & policy = solver_cfg_.fallback_on_failure;
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";

    if (policy == "hold_last" && has_last_command_) {
      cmd = last_command_;
      cmd.header.stamp = now();
    } else if (policy == "brake" && has_last_command_) {
      cmd.drive.speed = 0.0;
      cmd.drive.steering_angle = last_command_.drive.steering_angle;
    } else {
      cmd.drive.speed = 0.0;
      cmd.drive.steering_angle = 0.0;
    }

    drive_pub_->publish(cmd);
    u_prev_ = Input(cmd.drive.steering_angle, 0.0);
    debug_pub_->publishStatus(false, 0.0, 0.0, reason, now());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "fallback active (%s): %s", policy.c_str(), reason.c_str());
  }

  void controlLoop()
  {
    if (!has_odom_ || !ref_handler_.hasWaypoints()) {
      applyFallback(!has_odom_ ? "no odometry received yet" : "no raceline received yet");
      return;
    }

    constexpr double kStaleOdomTimeoutS = 0.5;
    if ((now() - last_odom_stamp_).seconds() > kStaleOdomTimeoutS) {
      applyFallback("stale odometry");
      return;
    }

    const MpcOutput out = controller_->computeCommand(current_state_, u_prev_, ref_handler_);
    if (!out.solved) {
      applyFallback("QP solve failed");
      return;
    }

    u_prev_ = out.u0;
    const double control_period_s = 1.0 / std::max(param_manager_.controlRateHz(), 1.0);
    const double speed_min = get_parameter("limits.speed_min").as_double();
    const double speed_max = get_parameter("limits.speed_max").as_double();
    const double speed_cmd = std::clamp(
      current_state_(3) + out.u0(1) * control_period_s, speed_min, speed_max);

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = out.u0(0);
    cmd.drive.speed = speed_cmd;
    drive_pub_->publish(cmd);
    last_command_ = cmd;
    has_last_command_ = true;

    debug_pub_->publishPredictedPath(out.predicted_states, "map", now());
    debug_pub_->publishStatus(true, out.solve_time_ms, out.cost, "OK", now());

    const ReferencePoint nearest = ref_handler_.nearestPoint(current_state_(0), current_state_(1));
    const double dx = current_state_(0) - nearest.x;
    const double dy = current_state_(1) - nearest.y;
    const double e_y = -std::sin(nearest.psi) * dx + std::cos(nearest.psi) * dy;
    const double e_psi = current_state_(2) - nearest.psi;
    debug_pub_->publishDebugScalars(e_y, e_psi, out.cost, out.solve_time_ms);
  }

  ParameterManager param_manager_;
  TopicsConfig topics_;
  SolverConfig solver_cfg_;
  std::unique_ptr<VehicleModel> vehicle_model_;
  std::unique_ptr<MpcController> controller_;
  ReferenceTrajectoryHandler ref_handler_;
  std::unique_ptr<DebugPublisher> debug_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f1tenth_msgs::msg::WaypointArray>::SharedPtr waypoint_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_{false};
  rclcpp::Time last_odom_stamp_;
  State current_state_{State::Zero()};
  Input u_prev_{Input::Zero()};

  bool has_last_command_{false};
  ackermann_msgs::msg::AckermannDriveStamped last_command_;
};

}  // namespace mpc_path_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpc_path_tracking::MpcNode>());
  rclcpp::shutdown();
  return 0;
}
