#include "mpc_path_tracking/debug_publisher.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mpc_path_tracking
{

DebugPublisher::DebugPublisher(
  rclcpp::Node * node, const TopicsConfig & topics, const DebugConfig & debug_cfg)
: node_(node), debug_cfg_(debug_cfg)
{
  status_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
    topics.status_topic, rclcpp::QoS(1));

  if (!debug_cfg_.enabled) {
    return;
  }

  if (debug_cfg_.publish_predicted_path) {
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      topics.predicted_trajectory_topic, rclcpp::QoS(1));
  }

  e_y_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    topics.debug_topic_prefix + "/lateral_error", rclcpp::QoS(1));
  e_psi_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
    topics.debug_topic_prefix + "/heading_error", rclcpp::QoS(1));
  if (debug_cfg_.publish_cost_breakdown) {
    cost_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      topics.debug_topic_prefix + "/cost", rclcpp::QoS(1));
  }
  if (debug_cfg_.log_solve_time) {
    solve_time_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      topics.debug_topic_prefix + "/solve_time_ms", rclcpp::QoS(1));
  }
}

void DebugPublisher::publishPredictedPath(
  const std::vector<State> & states, const std::string & frame_id, const rclcpp::Time & stamp)
{
  if (!debug_cfg_.enabled || !debug_cfg_.publish_predicted_path || !path_pub_) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id;
  path.poses.reserve(states.size());
  for (const auto & x : states) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = x(0);
    pose.pose.position.y = x(1);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x(2));
    pose.pose.orientation = tf2::toMsg(q);
    path.poses.push_back(pose);
  }
  path_pub_->publish(path);
}

void DebugPublisher::publishStatus(
  bool solved, double solve_time_ms, double cost, const std::string & message,
  const rclcpp::Time & stamp)
{
  publishStatus(solved, solve_time_ms, cost, message, StatusInfo{}, stamp);
}

void DebugPublisher::publishStatus(
  bool solved, double solve_time_ms, double cost, const std::string & message,
  const StatusInfo & info, const rclcpp::Time & /*stamp*/)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "mpc_path_tracking";
  status.level = solved ?
    diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = message;

  diagnostic_msgs::msg::KeyValue solved_kv;
  solved_kv.key = "solved";
  solved_kv.value = solved ? "true" : "false";
  status.values.push_back(solved_kv);

  diagnostic_msgs::msg::KeyValue time_kv;
  time_kv.key = "solve_time_ms";
  time_kv.value = std::to_string(solve_time_ms);
  status.values.push_back(time_kv);

  diagnostic_msgs::msg::KeyValue cost_kv;
  cost_kv.key = "cost";
  cost_kv.value = std::to_string(cost);
  status.values.push_back(cost_kv);

  // The tire set the solver is ACTUALLY using. mpc/update_params bypasses the
  // ROS parameters entirely, so without this there is no way to tell from
  // outside whether sysid has pushed an identification, or what it was.
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value;
      status.values.push_back(kv);
    };
  add("tire_front_BCDE",
    std::to_string(info.tire.Bf) + "," + std::to_string(info.tire.Cf) + "," +
    std::to_string(info.tire.Df) + "," + std::to_string(info.tire.Ef));
  add("tire_rear_BCDE",
    std::to_string(info.tire.Br) + "," + std::to_string(info.tire.Cr) + "," +
    std::to_string(info.tire.Dr) + "," + std::to_string(info.tire.Er));
  add("infeasible_ref_stages", std::to_string(info.infeasible_ref_stages));
  add("relinearized", info.relinearized ? "true" : "false");
  add("x0_prediction_s", std::to_string(info.x0_prediction_s));

  status_pub_->publish(status);
}

void DebugPublisher::publishDebugScalars(double e_y, double e_psi, double cost, double solve_time_ms)
{
  if (!debug_cfg_.enabled) {
    return;
  }
  std_msgs::msg::Float64 msg;
  msg.data = e_y;
  e_y_pub_->publish(msg);
  msg.data = e_psi;
  e_psi_pub_->publish(msg);
  if (cost_pub_) {
    msg.data = cost;
    cost_pub_->publish(msg);
  }
  if (solve_time_pub_) {
    msg.data = solve_time_ms;
    solve_time_pub_->publish(msg);
  }
}

}  // namespace mpc_path_tracking
