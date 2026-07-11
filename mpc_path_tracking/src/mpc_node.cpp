#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "adaptive_controller_interfaces/srv/identified_param.hpp"
#include "f1tenth_msgs/msg/waypoint_array.hpp"
#include "mpc_path_tracking/debug_publisher.hpp"
#include "mpc_path_tracking/mpc_controller.hpp"
#include "mpc_path_tracking/parameter_manager.hpp"
#include "mpc_path_tracking/reference_trajectory_handler.hpp"
#include "mpc_path_tracking/solver_interface.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"
#include "track_geometry_utils/track_geometry_utils.hpp"

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
    standalone_mode_ = param_manager_.standaloneMode();

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

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      topics_.enable_topic, rclcpp::QoS(1),
      std::bind(&MpcNode::enableCallback, this, _1));

    // The param-update service must never contend with the control-loop
    // timer for time on the executor - it runs on its own reentrant
    // callback group so a pending/slow service call can't delay a solve.
    param_service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    param_service_ = create_service<adaptive_controller_interfaces::srv::IdentifiedParam>(
      topics_.param_service,
      std::bind(&MpcNode::updateParamsCallback, this, _1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      param_service_callback_group_);

    const double rate_hz = std::max(param_manager_.controlRateHz(), 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      std::bind(&MpcNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(), "mpc_path_tracking ready: odom=%s waypoints=%s drive=%s standalone=%s",
      topics_.odom_topic.c_str(), topics_.waypoint_topic.c_str(), topics_.drive_topic.c_str(),
      standalone_mode_ ? "true" : "false");
  }

private:
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    start_working_ = msg->data;
  }

  void updateParamsCallback(
    const std::shared_ptr<adaptive_controller_interfaces::srv::IdentifiedParam::Request> request,
    std::shared_ptr<adaptive_controller_interfaces::srv::IdentifiedParam::Response> response)
  {
    if (request->param_values.size() != 8) {
      RCLCPP_ERROR(
        get_logger(), "mpc/update_params: expected 8 param_values, got %zu",
        request->param_values.size());
      response->ack = false;
      return;
    }

    TireParams tire;
    tire.Bf = request->param_values[0];
    tire.Cf = request->param_values[1];
    tire.Df = request->param_values[2];
    tire.Ef = request->param_values[3];
    tire.Br = request->param_values[4];
    tire.Cr = request->param_values[5];
    tire.Dr = request->param_values[6];
    tire.Er = request->param_values[7];
    // controller_ holds its own VehicleModel copy (see MpcController::model_)
    // - that is the instance computeCommand() actually solves against, so
    // it must be updated directly rather than vehicle_model_ (which is only
    // ever used as the one-time construction template for that copy).
    controller_->setTireParams(tire);
    RCLCPP_INFO(get_logger(), "tire params updated via mpc/update_params");
    response->ack = true;
  }

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
    last_waypoints_msg_ = *msg;
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

    if (standalone_mode_ || start_working_) {
      drive_pub_->publish(cmd);
    }
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
    if (standalone_mode_ || start_working_) {
      drive_pub_->publish(cmd);
    }
    last_command_ = cmd;
    has_last_command_ = true;

    debug_pub_->publishPredictedPath(out.predicted_states, "map", now());
    debug_pub_->publishStatus(true, out.solve_time_ms, out.cost, "OK", now());

    const track_geometry_utils::TrackError track_error = track_geometry_utils::computeTrackError(
      last_waypoints_msg_, current_state_(0), current_state_(1), current_state_(2));
    debug_pub_->publishDebugScalars(
      track_error.e_y, track_error.heading_error, out.cost, out.solve_time_ms);
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::CallbackGroup::SharedPtr param_service_callback_group_;
  rclcpp::Service<adaptive_controller_interfaces::srv::IdentifiedParam>::SharedPtr param_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_{false};
  rclcpp::Time last_odom_stamp_;
  State current_state_{State::Zero()};
  Input u_prev_{Input::Zero()};
  f1tenth_msgs::msg::WaypointArray last_waypoints_msg_;

  bool has_last_command_{false};
  ackermann_msgs::msg::AckermannDriveStamped last_command_;

  bool standalone_mode_{false};
  bool start_working_{false};
};

}  // namespace mpc_path_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mpc_path_tracking::MpcNode>();
  // MultiThreadedExecutor so mpc/update_params (its own ReentrantCallbackGroup)
  // never contends with the control-loop timer's default callback group.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
