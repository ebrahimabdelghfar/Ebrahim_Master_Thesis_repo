#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/utils.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "adaptive_controller_interfaces/srv/identified_param.hpp"
#include "adaptive_controller_manager/parameter_manager.hpp"
#include "f1tenth_msgs/msg/waypoint_array.hpp"
#include "track_geometry_utils/track_geometry_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace adaptive_controller_manager
{

// Arbitrates the single /drive output between pure_pursuit and
// mpc_path_tracking, mediates tire-parameter handoff between On-Track-SysID
// and mpc_path_tracking, and drives the bootstrap -> identify -> handover ->
// fallback state machine described in enhanced_controller_plan.md.
class ManagerNode : public rclcpp::Node
{
public:
  ManagerNode()
  : rclcpp::Node("adaptive_controller_manager"), param_manager_(this)
  {
    param_manager_.declareAll();
    topics_ = param_manager_.topics();
    safety_ = param_manager_.safety();
    tire_bounds_ = param_manager_.tireBounds();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topics_.odom_topic, rclcpp::QoS(1), std::bind(&ManagerNode::odomCallback, this, _1));

    rclcpp::QoS waypoint_qos(1);
    waypoint_qos.transient_local();
    waypoint_sub_ = create_subscription<f1tenth_msgs::msg::WaypointArray>(
      topics_.waypoint_topic, waypoint_qos, std::bind(&ManagerNode::waypointCallback, this, _1));

    pp_drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      topics_.pp_drive_topic, rclcpp::QoS(1), std::bind(&ManagerNode::ppDriveCallback, this, _1));
    mpc_drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      topics_.mpc_drive_topic, rclcpp::QoS(1), std::bind(&ManagerNode::mpcDriveCallback, this, _1));

    pp_state_sub_ = create_subscription<std_msgs::msg::Bool>(
      topics_.pp_state_topic, rclcpp::QoS(1), std::bind(&ManagerNode::ppStateCallback, this, _1));
    pp_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      topics_.pp_health_topic, rclcpp::QoS(1), std::bind(&ManagerNode::ppHealthCallback, this, _1));
    mpc_status_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
      topics_.mpc_status_topic, rclcpp::QoS(1),
      std::bind(&ManagerNode::mpcStatusCallback, this, _1));

    rclcpp::QoS first_run_qos(1);
    first_run_qos.transient_local();
    sysid_first_run_sub_ = create_subscription<std_msgs::msg::Bool>(
      topics_.sysid_first_run_topic, first_run_qos,
      std::bind(&ManagerNode::sysidFirstRunCallback, this, _1));

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      topics_.drive_topic, rclcpp::QoS(1));
    start_working_pp_pub_ = create_publisher<std_msgs::msg::Bool>(
      topics_.start_working_pp_topic, rclcpp::QoS(1));
    start_working_mpc_pub_ = create_publisher<std_msgs::msg::Bool>(
      topics_.start_working_mpc_topic, rclcpp::QoS(1));

    // Observational only (Part A) - lets a test script or rqt tell the FSM
    // state apart and see the same error signals the arming gate reads,
    // without changing any control-loop behavior.
    state_pub_ = create_publisher<std_msgs::msg::String>("manager/state", rclcpp::QoS(1));
    e_y_debug_pub_ = create_publisher<std_msgs::msg::Float64>(
      "manager/debug/lateral_error", rclcpp::QoS(1));
    heading_error_debug_pub_ = create_publisher<std_msgs::msg::Float64>(
      "manager/debug/heading_error", rclcpp::QoS(1));

    // RViz visualization of which controller is currently active - a
    // colored sphere + text label hovering above the vehicle, so it's
    // visible at a glance without reading /manager/state on the console.
    active_controller_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "manager/active_controller_marker", rclcpp::QoS(1));

    // sysid/update_params server and mpc/update_params client each get their
    // own reentrant callback group so neither a slow/pending forward to MPC
    // nor a SysID submission can delay the arbitration timer, which stays on
    // the node's default (mutually-exclusive) group alongside every
    // subscription above.
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    client_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    sysid_update_params_srv_ =
      create_service<adaptive_controller_interfaces::srv::IdentifiedParam>(
      topics_.sysid_update_params_service,
      std::bind(&ManagerNode::onSysidUpdateParams, this, _1, _2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      service_callback_group_);

    mpc_update_params_client_ =
      create_client<adaptive_controller_interfaces::srv::IdentifiedParam>(
      topics_.mpc_update_params_service,
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      client_callback_group_);

    const double rate_hz = std::max(safety_.control_rate_hz, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), std::bind(&ManagerNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(), "adaptive_controller_manager ready, starting in BOOTSTRAP_PP");
  }

private:
  enum class FsmState
  {
    BOOTSTRAP_PP,
    RUNNING_PP,
    SWITCHING_TO_MPC,
    RUNNING_MPC,
    SWITCHING_TO_PP,
    EMERGENCY_HALT
  };

  // ---------------- Subscription callbacks (default callback group) ----------------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
    has_odom_ = true;
    last_odom_stamp_ = now();
  }

  void waypointCallback(const f1tenth_msgs::msg::WaypointArray::SharedPtr msg)
  {
    last_waypoints_ = *msg;
    has_waypoints_ = true;
  }

  void ppDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    last_pp_cmd_ = *msg;
    has_pp_cmd_ = true;
    last_pp_cmd_stamp_ = now();
  }

  void mpcDriveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    last_mpc_cmd_ = *msg;
    has_mpc_cmd_ = true;
    last_mpc_cmd_stamp_ = now();
  }

  void ppStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    pp_state_ = msg->data;
    has_pp_state_ = true;
    last_pp_state_stamp_ = now();
  }

  void ppHealthCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    pp_health_ = msg->data;
  }

  void mpcStatusCallback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
  {
    mpc_health_ = (msg->level == diagnostic_msgs::msg::DiagnosticStatus::OK);
    has_mpc_status_ = true;
    last_mpc_status_stamp_ = now();
  }

  void sysidFirstRunCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    sysid_first_run_ = msg->data;
  }

  // ---------------- sysid/update_params server (reentrant group) ----------------

  void onSysidUpdateParams(
    const std::shared_ptr<adaptive_controller_interfaces::srv::IdentifiedParam::Request> request,
    std::shared_ptr<adaptive_controller_interfaces::srv::IdentifiedParam::Response> response)
  {
    if (request->param_values.size() != 8) {
      RCLCPP_ERROR(
        get_logger(), "sysid/update_params: expected 8 param_values, got %zu",
        request->param_values.size());
      response->ack = false;
      return;
    }

    for (size_t i = 0; i < 8; ++i) {
      const double v = request->param_values[i];
      if (v < tire_bounds_.min[i] || v > tire_bounds_.max[i]) {
        RCLCPP_WARN(
          get_logger(),
          "sysid/update_params: param[%zu]=%.4f outside plausible bounds [%.4f, %.4f] - rejected",
          i, v, tire_bounds_.min[i], tire_bounds_.max[i]);
        response->ack = false;
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      for (size_t i = 0; i < 8; ++i) {
        stored_tire_params_[i] = request->param_values[i];
      }
      ++stored_version_;
    }
    RCLCPP_INFO(get_logger(), "sysid/update_params: accepted and stored new tire params");
    response->ack = true;
  }

  // ---------------- Control loop (default callback group, timer) ----------------

  void controlLoop()
  {
    updateTrackError();
    updateConvergenceHistory();

    switch (state_) {
      case FsmState::BOOTSTRAP_PP:
        if (pp_state_) {
          RCLCPP_INFO(get_logger(), "PP confirmed active - RUNNING_PP");
          state_ = FsmState::RUNNING_PP;
        }
        break;

      case FsmState::RUNNING_PP:
        // PP is the sole ACTIVE controller here - mpc_path_tracking keeps
        // solving and publishing /mpc/status even while gated off
        // (Start_Working_mpc=false), so "mpc_health_ == true" only means
        // its solver works, not that it's a validated, instantly-usable
        // fallback. Promoting it without the normal arming gates would
        // skip the safety checks entirely, so if PP fails here, go
        // straight to EMERGENCY_HALT regardless of MPC's idle solver
        // health (caught via simulation testing - see docs).
        if (!ppHealthOk()) {
          RCLCPP_ERROR(get_logger(), "PP unhealthy in RUNNING_PP - EMERGENCY_HALT");
          state_ = FsmState::EMERGENCY_HALT;
        } else {
          tryForwardStoredParams();
        }
        break;

      case FsmState::SWITCHING_TO_MPC:
        stepSwitching(true);
        break;

      case FsmState::RUNNING_MPC:
        if (!mpcHealthOk()) {
          if (ppHealthOk()) {
            RCLCPP_WARN(get_logger(), "MPC unhealthy in RUNNING_MPC - switching to PP");
            beginSwitch(false);
          } else {
            RCLCPP_ERROR(
              get_logger(), "MPC unhealthy in RUNNING_MPC and PP also unhealthy - EMERGENCY_HALT");
            state_ = FsmState::EMERGENCY_HALT;
          }
        } else {
          tryForwardStoredParams();
        }
        break;

      case FsmState::SWITCHING_TO_PP:
        stepSwitching(false);
        break;

      case FsmState::EMERGENCY_HALT:
        if (ppHealthOk() || mpcHealthOk()) {
          RCLCPP_INFO(get_logger(), "A controller recovered - returning to BOOTSTRAP_PP");
          state_ = FsmState::BOOTSTRAP_PP;
        }
        break;
    }

    publishEnableFlags();
    publishDebugTopics();
    publishActiveControllerMarker();
    logStatus();
    const auto cmd = computeOutput();
    drive_pub_->publish(cmd);
  }

  static const char * fsmStateName(FsmState state)
  {
    switch (state) {
      case FsmState::BOOTSTRAP_PP: return "BOOTSTRAP_PP";
      case FsmState::RUNNING_PP: return "RUNNING_PP";
      case FsmState::SWITCHING_TO_MPC: return "SWITCHING_TO_MPC";
      case FsmState::RUNNING_MPC: return "RUNNING_MPC";
      case FsmState::SWITCHING_TO_PP: return "SWITCHING_TO_PP";
      case FsmState::EMERGENCY_HALT: return "EMERGENCY_HALT";
      default: return "UNKNOWN";
    }
  }

  void publishDebugTopics()
  {
    std_msgs::msg::String state_msg;
    state_msg.data = fsmStateName(state_);
    state_pub_->publish(state_msg);

    if (has_track_error_) {
      std_msgs::msg::Float64 e_y_msg;
      e_y_msg.data = last_e_y_;
      e_y_debug_pub_->publish(e_y_msg);

      std_msgs::msg::Float64 heading_msg;
      heading_msg.data = last_heading_error_;
      heading_error_debug_pub_->publish(heading_msg);
    }
  }

  // Text + color per FSM state, shared by the RViz marker and status logs.
  static void activeControllerLabel(FsmState state, std::string & text, float & r, float & g, float & b)
  {
    switch (state) {
      case FsmState::BOOTSTRAP_PP:
        text = "BOOTSTRAP"; r = 0.6f; g = 0.6f; b = 0.6f; return;
      case FsmState::RUNNING_PP:
        text = "PURE PURSUIT"; r = 0.0f; g = 1.0f; b = 0.0f; return;
      case FsmState::SWITCHING_TO_MPC:
        text = "SWITCHING -> MPC"; r = 1.0f; g = 0.6f; b = 0.0f; return;
      case FsmState::RUNNING_MPC:
        text = "MPC"; r = 0.0f; g = 0.4f; b = 1.0f; return;
      case FsmState::SWITCHING_TO_PP:
        text = "SWITCHING -> PP"; r = 1.0f; g = 0.6f; b = 0.0f; return;
      case FsmState::EMERGENCY_HALT:
        text = "EMERGENCY HALT"; r = 1.0f; g = 0.0f; b = 0.0f; return;
    }
    text = "UNKNOWN"; r = 1.0f; g = 1.0f; b = 1.0f;
  }

  void publishActiveControllerMarker()
  {
    if (!has_odom_) {
      return;
    }
    std::string text;
    float r, g, b;
    activeControllerLabel(state_, text, r, g, b);

    const auto stamp = now();
    const auto lifetime = rclcpp::Duration::from_seconds(0.5);
    const double x = odom_.pose.pose.position.x;
    const double y = odom_.pose.pose.position.y;

    visualization_msgs::msg::Marker dot;
    dot.header.frame_id = "map";
    dot.header.stamp = stamp;
    dot.ns = "adaptive_controller_manager";
    dot.id = 0;
    dot.type = visualization_msgs::msg::Marker::SPHERE;
    dot.action = visualization_msgs::msg::Marker::ADD;
    dot.pose.position.x = x;
    dot.pose.position.y = y;
    dot.pose.position.z = 0.5;
    dot.pose.orientation.w = 1.0;
    dot.scale.x = dot.scale.y = dot.scale.z = 0.4;
    dot.color.r = r; dot.color.g = g; dot.color.b = b; dot.color.a = 0.9f;
    dot.lifetime = lifetime;

    visualization_msgs::msg::Marker label;
    label.header = dot.header;
    label.ns = "adaptive_controller_manager";
    label.id = 1;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = x;
    label.pose.position.y = y;
    label.pose.position.z = 1.1;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.5;
    label.color.r = r; label.color.g = g; label.color.b = b; label.color.a = 1.0f;
    label.text = text;
    label.lifetime = lifetime;

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(dot);
    markers.markers.push_back(label);
    active_controller_marker_pub_->publish(markers);
  }

  // Comma-separated list of arming gates currently failing, for the
  // throttled diagnostic log below - "all gates pass" when none are.
  std::string armingDiagnostics() const
  {
    std::vector<std::string> failing;
    if (!odomFresh()) {failing.push_back("odom_stale");}
    if (!has_track_error_) {failing.push_back("no_track_error");}
    if (has_odom_ && odom_.twist.twist.linear.x <= safety_.v_min) {failing.push_back("v_min");}
    if (has_track_error_ && std::abs(last_e_y_) >= safety_.e_y_max) {failing.push_back("e_y_max");}
    if (has_track_error_ && std::abs(last_heading_error_) >= safety_.theta_max) {
      failing.push_back("theta_max");
    }
    if (!convergenceOk()) {failing.push_back("convergence");}
    if (failing.empty()) {
      return "all gates pass";
    }
    std::string joined;
    for (size_t i = 0; i < failing.size(); ++i) {
      joined += failing[i];
      if (i + 1 < failing.size()) {joined += ",";}
    }
    return joined;
  }

  // Periodic (throttled) flow/status summary covering both controllers, so
  // the manager's own log is enough to follow what's happening without
  // cross-referencing pure_pursuit's/mpc_path_tracking's separate logs.
  void logStatus()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[status] state=%s | pp: health=%s state=%s | mpc: health=%s | v_x=%.2f e_y=%.3f theta=%.3f | "
      "params: stored_v=%lu fwd_v=%lu pending=%s",
      fsmStateName(state_),
      ppHealthOk() ? "ok" : "BAD", pp_state_ ? "active" : "idle",
      mpcHealthOk() ? "ok" : "BAD",
      has_odom_ ? odom_.twist.twist.linear.x : 0.0, last_e_y_, last_heading_error_,
      static_cast<unsigned long>(stored_version_), static_cast<unsigned long>(forwarded_version_),
      mpc_forward_pending_.load() ? "yes" : "no");

    if (state_ == FsmState::RUNNING_PP && stored_version_ != forwarded_version_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "[status] stored params awaiting arming: %s", armingDiagnostics().c_str());
    }
  }

  // to_mpc=true: currently SWITCHING_TO_MPC; false: SWITCHING_TO_PP.
  void stepSwitching(bool to_mpc)
  {
    const double elapsed = (now() - switch_start_time_).seconds();
    const bool incoming_healthy = to_mpc ? mpcHealthOk() : ppHealthOk();

    if (incoming_cmd_seen_since_switch_ && !incoming_healthy) {
      RCLCPP_WARN(
        get_logger(), "%s unhealthy mid-switch - aborting back to %s",
        to_mpc ? "MPC" : "PP", to_mpc ? "PP" : "MPC");
      state_ = to_mpc ? FsmState::RUNNING_PP : FsmState::RUNNING_MPC;
      return;
    }

    if (elapsed < safety_.delta_t_switch) {
      return;
    }

    if (incoming_healthy) {
      RCLCPP_INFO(get_logger(), "Switch to %s complete", to_mpc ? "MPC" : "PP");
      state_ = to_mpc ? FsmState::RUNNING_MPC : FsmState::RUNNING_PP;
    } else if (to_mpc) {
      RCLCPP_WARN(get_logger(), "Switch window elapsed, MPC unhealthy - reverting to PP");
      state_ = FsmState::RUNNING_PP;
    } else {
      RCLCPP_ERROR(get_logger(), "Switch window elapsed, PP also unhealthy - EMERGENCY_HALT");
      state_ = FsmState::EMERGENCY_HALT;
    }
  }

  // to_mpc=true: begin RUNNING_PP -> SWITCHING_TO_MPC; false: the reverse.
  void beginSwitch(bool to_mpc)
  {
    v_frozen_ = to_mpc ?
      (has_pp_cmd_ ? last_pp_cmd_.drive.speed : 0.0) :
      (has_mpc_cmd_ ? last_mpc_cmd_.drive.speed : 0.0);
    switch_start_time_ = now();
    incoming_cmd_seen_since_switch_ = false;
    state_ = to_mpc ? FsmState::SWITCHING_TO_MPC : FsmState::SWITCHING_TO_PP;
    RCLCPP_INFO(
      get_logger(), "Beginning switch to %s (v_frozen=%.2f)", to_mpc ? "MPC" : "PP", v_frozen_);
  }

  // Consumes any pending mpc/update_params ack, then forwards the latest
  // stored (validated) tire params if: not already forwarding, there is a
  // new version to send, and either we're already RUNNING_MPC (continuous
  // re-identification, no arming gate) or the initial PP->MPC handover's
  // safety gates all pass.
  void tryForwardStoredParams()
  {
    if (mpc_forward_last_ack_.exchange(false)) {
      const uint64_t acked_version = mpc_forward_last_ack_version_.load();
      forwarded_version_ = acked_version;
      if (state_ == FsmState::RUNNING_PP) {
        beginSwitch(true);
        return;
      }
    }

    if (mpc_forward_pending_.load()) {
      return;
    }

    std::array<float, 8> params{};
    uint64_t version = 0;
    bool have_new = false;
    {
      std::lock_guard<std::mutex> lock(params_mutex_);
      if (stored_version_ != forwarded_version_) {
        params = stored_tire_params_;
        version = stored_version_;
        have_new = true;
      }
    }
    if (!have_new) {
      return;
    }

    if (state_ == FsmState::RUNNING_PP && !armingConditionsSatisfied()) {
      return;
    }

    auto request =
      std::make_shared<adaptive_controller_interfaces::srv::IdentifiedParam::Request>();
    std::copy(params.begin(), params.end(), request->param_values.begin());
    mpc_forward_pending_.store(true);
    mpc_update_params_client_->async_send_request(
      request,
      [this, version](
        rclcpp::Client<adaptive_controller_interfaces::srv::IdentifiedParam>::SharedFuture future) {
        auto response = future.get();
        const bool ack = response && response->ack;
        if (!ack) {
          RCLCPP_WARN(get_logger(), "mpc/update_params was not acked by mpc_path_tracking");
        }
        mpc_forward_last_ack_version_.store(version);
        mpc_forward_last_ack_.store(ack);
        mpc_forward_pending_.store(false);
      });
  }

  bool armingConditionsSatisfied() const
  {
    if (!odomFresh() || !has_track_error_) {
      return false;
    }
    if (odom_.twist.twist.linear.x <= safety_.v_min) {
      return false;
    }
    if (std::abs(last_e_y_) >= safety_.e_y_max) {
      return false;
    }
    if (std::abs(last_heading_error_) >= safety_.theta_max) {
      return false;
    }
    return convergenceOk();
  }

  bool convergenceOk() const
  {
    if (static_cast<int>(e_y_rate_history_.size()) < safety_.error_convergence_window) {
      return false;
    }
    double sum = 0.0;
    for (const double v : e_y_rate_history_) {
      sum += v;
    }
    return (sum / static_cast<double>(e_y_rate_history_.size())) < 0.0;
  }

  void updateTrackError()
  {
    has_track_error_ = false;
    if (!has_odom_ || !has_waypoints_ || last_waypoints_.waypoints.empty()) {
      return;
    }
    const double yaw = tf2::getYaw(odom_.pose.pose.orientation);
    const auto err = track_geometry_utils::computeTrackError(
      last_waypoints_, odom_.pose.pose.position.x, odom_.pose.pose.position.y, yaw);
    last_e_y_ = err.e_y;
    last_heading_error_ = err.heading_error;
    has_track_error_ = true;
  }

  void updateConvergenceHistory()
  {
    if (!has_track_error_) {
      return;
    }
    const rclcpp::Time now_time = now();
    if (has_prev_e_y_) {
      const double dt = (now_time - prev_e_y_stamp_).seconds();
      if (dt > 1e-6) {
        e_y_rate_history_.push_back((last_e_y_ - prev_e_y_) / dt);
        while (static_cast<int>(e_y_rate_history_.size()) > safety_.error_convergence_window) {
          e_y_rate_history_.pop_front();
        }
      }
    }
    prev_e_y_ = last_e_y_;
    prev_e_y_stamp_ = now_time;
    has_prev_e_y_ = true;
  }

  bool odomFresh() const
  {
    return has_odom_ && (now() - last_odom_stamp_).seconds() < safety_.delta_t_state_max;
  }

  bool ppFresh() const
  {
    return has_pp_cmd_ && (now() - last_pp_cmd_stamp_).seconds() < safety_.delta_t_timeout;
  }

  bool mpcFresh() const
  {
    return has_mpc_cmd_ && (now() - last_mpc_cmd_stamp_).seconds() < safety_.delta_t_timeout;
  }

  // Safety 5A: an active controller whose *_state/status stops updating for
  // delta_t_timeout is treated as unhealthy, on top of its own health flag.
  bool ppHealthOk() const
  {
    return pp_health_ && has_pp_state_ &&
           (now() - last_pp_state_stamp_).seconds() < safety_.delta_t_timeout;
  }

  bool mpcHealthOk() const
  {
    return mpc_health_ && has_mpc_status_ &&
           (now() - last_mpc_status_stamp_).seconds() < safety_.delta_t_timeout;
  }

  void publishEnableFlags()
  {
    const bool pp_on = (
      state_ == FsmState::BOOTSTRAP_PP || state_ == FsmState::RUNNING_PP ||
      state_ == FsmState::SWITCHING_TO_PP);
    const bool mpc_on = (state_ == FsmState::SWITCHING_TO_MPC || state_ == FsmState::RUNNING_MPC);

    std_msgs::msg::Bool pp_msg;
    pp_msg.data = pp_on;
    start_working_pp_pub_->publish(pp_msg);

    std_msgs::msg::Bool mpc_msg;
    mpc_msg.data = mpc_on;
    start_working_mpc_pub_->publish(mpc_msg);
  }

  ackermann_msgs::msg::AckermannDriveStamped computeOutput()
  {
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";

    switch (state_) {
      case FsmState::BOOTSTRAP_PP:
      case FsmState::RUNNING_PP:
        if (ppFresh()) {
          cmd = last_pp_cmd_;
          cmd.header.stamp = now();
        }
        break;

      case FsmState::RUNNING_MPC:
        if (mpcFresh()) {
          cmd = last_mpc_cmd_;
          cmd.header.stamp = now();
        }
        break;

      case FsmState::SWITCHING_TO_MPC:
      case FsmState::SWITCHING_TO_PP: {
        const bool to_mpc = (state_ == FsmState::SWITCHING_TO_MPC);
        const bool incoming_fresh = to_mpc ? mpcFresh() : ppFresh();
        const rclcpp::Time incoming_stamp = to_mpc ? last_mpc_cmd_stamp_ : last_pp_cmd_stamp_;
        if (incoming_fresh && incoming_stamp >= switch_start_time_) {
          incoming_cmd_seen_since_switch_ = true;
        }

        if (!incoming_cmd_seen_since_switch_) {
          // Bridge the gap before the newly-activated controller's first
          // command arrives (its node hasn't processed Start_Working_* yet)
          // - hold the previous output rather than publish steering=0 or a
          // stale command.
          cmd = last_output_cmd_;
          cmd.header.stamp = now();
          break;
        }

        const auto & incoming_cmd = to_mpc ? last_mpc_cmd_ : last_pp_cmd_;
        const double elapsed = (now() - switch_start_time_).seconds();
        const double alpha =
          std::clamp(elapsed / std::max(safety_.delta_t_switch, 1e-3), 0.0, 1.0);
        cmd.drive.steering_angle = incoming_cmd.drive.steering_angle;
        cmd.drive.speed = (1.0 - alpha) * v_frozen_ + alpha * incoming_cmd.drive.speed;
        break;
      }

      case FsmState::EMERGENCY_HALT:
      default:
        cmd.drive.speed = 0.0;
        cmd.drive.steering_angle = 0.0;
        break;
    }

    // Any decrease from the previous tick's published speed is rate-limited
    // to a physically realistic deceleration - covers the ramp blend above,
    // EMERGENCY_HALT's hardcoded zero, and the stale-data fallback (both of
    // which would otherwise be an instant full stop). Increases are left
    // alone: the ramp already blends smoothly on the way up, and each
    // controller's own accel limits apply otherwise.
    if (cmd.drive.speed < last_output_cmd_.drive.speed) {
      const double dt = 1.0 / std::max(safety_.control_rate_hz, 1.0);
      const double min_allowed = last_output_cmd_.drive.speed - safety_.max_decel_mps2 * dt;
      cmd.drive.speed = static_cast<float>(std::max(static_cast<double>(cmd.drive.speed), min_allowed));
    }

    last_output_cmd_ = cmd;
    return cmd;
  }

  ParameterManager param_manager_;
  TopicsConfig topics_;
  SafetyConfig safety_;
  TireBounds tire_bounds_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f1tenth_msgs::msg::WaypointArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pp_drive_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr mpc_drive_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pp_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pp_health_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr mpc_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sysid_first_run_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_working_pp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_working_mpc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr e_y_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr heading_error_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr active_controller_marker_pub_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::Service<adaptive_controller_interfaces::srv::IdentifiedParam>::SharedPtr
    sysid_update_params_srv_;
  rclcpp::Client<adaptive_controller_interfaces::srv::IdentifiedParam>::SharedPtr
    mpc_update_params_client_;

  rclcpp::TimerBase::SharedPtr timer_;

  FsmState state_{FsmState::BOOTSTRAP_PP};

  bool has_odom_{false};
  nav_msgs::msg::Odometry odom_;
  rclcpp::Time last_odom_stamp_;

  bool has_waypoints_{false};
  f1tenth_msgs::msg::WaypointArray last_waypoints_;

  bool has_pp_cmd_{false};
  ackermann_msgs::msg::AckermannDriveStamped last_pp_cmd_;
  rclcpp::Time last_pp_cmd_stamp_;

  bool has_mpc_cmd_{false};
  ackermann_msgs::msg::AckermannDriveStamped last_mpc_cmd_;
  rclcpp::Time last_mpc_cmd_stamp_;

  bool pp_state_{false};
  bool has_pp_state_{false};
  rclcpp::Time last_pp_state_stamp_;
  bool pp_health_{false};

  bool mpc_health_{false};
  bool has_mpc_status_{false};
  rclcpp::Time last_mpc_status_stamp_;

  bool sysid_first_run_{true};

  bool has_track_error_{false};
  double last_e_y_{0.0};
  double last_heading_error_{0.0};
  bool has_prev_e_y_{false};
  double prev_e_y_{0.0};
  rclcpp::Time prev_e_y_stamp_;
  std::deque<double> e_y_rate_history_;

  ackermann_msgs::msg::AckermannDriveStamped last_output_cmd_;
  double v_frozen_{0.0};
  rclcpp::Time switch_start_time_;
  bool incoming_cmd_seen_since_switch_{false};

  std::mutex params_mutex_;
  std::array<float, 8> stored_tire_params_{};
  uint64_t stored_version_{0};
  uint64_t forwarded_version_{0};
  std::atomic<bool> mpc_forward_pending_{false};
  std::atomic<bool> mpc_forward_last_ack_{false};
  std::atomic<uint64_t> mpc_forward_last_ack_version_{0};
};

}  // namespace adaptive_controller_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<adaptive_controller_manager::ManagerNode>();
  // MultiThreadedExecutor: sysid/update_params (server) and mpc/update_params
  // (client) each run on their own ReentrantCallbackGroup so neither ever
  // delays the arbitration timer, which stays on the default group.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
