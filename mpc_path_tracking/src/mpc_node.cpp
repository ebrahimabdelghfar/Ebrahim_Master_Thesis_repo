#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
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
    param_manager_.printAll();
    topics_ = param_manager_.topics();
    solver_cfg_ = param_manager_.solverConfig();
    standalone_mode_ = param_manager_.standaloneMode();

    const VehicleParams vehicle_params = param_manager_.vehicleParams();
    const TireParams tire_params = param_manager_.tireParams();
    vehicle_model_ = std::make_unique<VehicleModel>(vehicle_params, tire_params);

    std::unique_ptr<SolverInterface> solver;
    if (solver_cfg_.backend == "acados") {
      solver = std::make_unique<AcadosMpcSolver>(param_manager_.mpcConfig().N, solver_cfg_.acados);
    } else {
      if (solver_cfg_.backend != "osqp") {
        RCLCPP_ERROR(
          get_logger(), "unknown solver.backend '%s', falling back to osqp",
          solver_cfg_.backend.c_str());
      }
      OsqpSolverSettings osqp_settings;
      osqp_settings.max_iter = solver_cfg_.max_iter;
      osqp_settings.eps_abs = solver_cfg_.eps_abs;
      osqp_settings.eps_rel = solver_cfg_.eps_rel;
      osqp_settings.warm_start = solver_cfg_.warm_start;
      osqp_settings.polish = solver_cfg_.polish;
      osqp_settings.time_limit_s = solver_cfg_.time_limit_ms / 1000.0;
      solver = std::make_unique<OsqpMpcSolver>(osqp_settings);
    }

    controller_ = std::make_unique<MpcController>(
      *vehicle_model_, std::move(solver), param_manager_.mpcConfig());

    debug_pub_ = std::make_unique<DebugPublisher>(this, topics_, param_manager_.debugConfig());

    // Request BEST_EFFORT on /odom - a bare rclcpp::QoS(1) requests RELIABLE,
    // which never matches the best-effort odom publisher (silent: no data, one
    // "incompatible QoS ... RELIABILITY" warning). Best-effort requests still
    // match reliable publishers.
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topics_.odom_topic, rclcpp::QoS(1).best_effort(),
      std::bind(&MpcNode::odomCallback, this, _1));

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
      get_logger(), "mpc_path_tracking ready: odom=%s waypoints=%s drive=%s standalone=%s backend=%s",
      topics_.odom_topic.c_str(), topics_.waypoint_topic.c_str(), topics_.drive_topic.c_str(),
      standalone_mode_ ? "true" : "false", solver_cfg_.backend.c_str());
    logModelStability(tire_params);
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
    // Reject a set that makes the prediction model open-loop unstable inside
    // the speed range we are actually going to drive. Above the oversteer
    // critical speed every horizon stage linearized there diverges, and over
    // N stages that is rho^N in the condensed QP - the solver can only report
    // it as a generic failure. Keeping the last physically sane model is
    // strictly better than accepting one we know cannot be solved.
    // Reject a set whose tires cannot generate the lateral acceleration the
    // raceline demands. D is the dimensionless peak friction coefficient, so
    // the model's grip ceiling is (Df*Fz_f + Dr*Fz_r)/m. If that is below the
    // reference's peak vx^2*kappa the model believes most corners are simply
    // impossible: steadyStateCornering finds no solution for those stages and
    // the MPC plans against a vehicle that cannot do the task. Measured with
    // the identified set (Df 0.4, Dr 0.4046 -> 0.40 g) against traj_race_cl.csv
    // at 27 m/s: 29461 infeasible horizon stages and 41.89 m of cross-track
    // error, versus 0 stages and 0.03 m on the same run with the startup prior.
    // A degenerate fit with D railed at its bound is the usual cause.
    if (ref_handler_.hasWaypoints() && ref_handler_.maxLateralDemand() > 0.0) {
      double fz_f = 0.0, fz_r = 0.0;
      vehicle_model_->normalLoads(fz_f, fz_r);
      const double grip_ceiling =
        (std::abs(tire.Df) * fz_f + std::abs(tire.Dr) * fz_r) /
        vehicle_model_->vehicleParams().mass;
      const double demand = ref_handler_.maxLateralDemand();
      if (grip_ceiling < demand) {
        RCLCPP_ERROR(
          get_logger(),
          "mpc/update_params REJECTED: identified tires give a peak lateral acceleration of "
          "%.2f m/s^2 (%.2f g, Df=%.3f Dr=%.3f) but the raceline demands %.2f m/s^2 (%.2f g). "
          "The model would treat most corners as infeasible. Keeping the previous tire params - "
          "check the sysid fit for parameters railed on their bounds.",
          grip_ceiling, grip_ceiling / 9.81, tire.Df, tire.Dr, demand, demand / 9.81);
        response->ack = false;
        return;
      }
    }

    const double v_crit = criticalSpeed(tire);
    const double v_max = get_parameter("limits.speed_max").as_double();
    if (v_crit < v_max) {
      RCLCPP_ERROR(
        get_logger(),
        "mpc/update_params REJECTED: identified tire set is oversteering with critical speed "
        "%.1f m/s, below limits.speed_max=%.1f m/s. The prediction model would be open-loop "
        "unstable over most of the horizon. Keeping the previous tire params.",
        v_crit, v_max);
      logModelStability(tire);
      response->ack = false;
      return;
    }

    // controller_ holds its own VehicleModel copy (see MpcController::model_)
    // - that is the instance computeCommand() actually solves against, so
    // it must be updated directly rather than vehicle_model_ (which is only
    // ever used as the one-time construction template for that copy).
    controller_->setTireParams(tire);
    RCLCPP_INFO(get_logger(), "tire params updated via mpc/update_params");
    logModelStability(tire);
    response->ack = true;
  }

  // Oversteer critical speed of the linear single-track model for `tire`, or
  // infinity when the axle balance is understeering (stable at every speed).
  double criticalSpeed(const TireParams & tire) const
  {
    const VehicleParams & vp = vehicle_model_->vehicleParams();
    double fz_f = 0.0, fz_r = 0.0;
    vehicle_model_->normalLoads(fz_f, fz_r);
    const double c_front = fz_f * tire.Bf * tire.Cf * tire.Df;
    const double c_rear = fz_r * tire.Br * tire.Cr * tire.Dr;
    const double margin = vp.l_f * c_front - vp.l_r * c_rear;
    if (margin <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }
    return (vp.l_f + vp.l_r) * std::sqrt(c_front * c_rear / (vp.mass * margin));
  }

  // The MPC linearizes about the *reference* at every stage, so an
  // oversteering tire set makes each stage Jacobian open-loop unstable above
  // the critical speed and the horizon amplifies that by rho^N - which the
  // QP solver can only report as a generic failure. Logging the critical
  // speed at the moment params change makes that visible up front.
  void logModelStability(const TireParams & tire)
  {
    const VehicleParams & vp = vehicle_model_->vehicleParams();
    double fz_f = 0.0, fz_r = 0.0;
    vehicle_model_->normalLoads(fz_f, fz_r);
    // d/dalpha of the Magic Formula at alpha = 0 is Fz*B*C*D (the E term
    // cancels there), i.e. the axle's linear cornering stiffness.
    const double c_front = fz_f * tire.Bf * tire.Cf * tire.Df;
    const double c_rear = fz_r * tire.Br * tire.Cr * tire.Dr;
    const double wheelbase = vp.l_f + vp.l_r;
    // Linear single-track stability: det(A_lat) > 0 <=> Cf*Cr*L^2/(m*v^2) >
    // l_f*Cf - l_r*Cr. Non-positive right-hand side => understeering, stable
    // at every speed.
    const double stability_margin = vp.l_f * c_front - vp.l_r * c_rear;
    if (stability_margin <= 0.0) {
      RCLCPP_INFO(
        get_logger(),
        "model check: C_front=%.0f N/rad C_rear=%.0f N/rad -> understeering, "
        "linearization stable at all speeds", c_front, c_rear);
      return;
    }
    const double v_crit =
      wheelbase * std::sqrt(c_front * c_rear / (vp.mass * stability_margin));
    RCLCPP_WARN(
      get_logger(),
      "model check: C_front=%.0f N/rad C_rear=%.0f N/rad -> OVERSTEERING, prediction model is "
      "open-loop unstable above v_crit=%.1f m/s; every horizon stage linearized above that "
      "speed diverges and the QP becomes ill-conditioned", c_front, c_rear, v_crit);
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
    last_waypoints_msg_ = *msg;
    ingestWaypoints();
  }

  // Re-clamps and re-loads the last received raceline against the CURRENT
  // limits.speed_max. Split out of waypointCallback so a runtime
  // `ros2 param set .. limits.speed_max` takes effect immediately: the
  // raceline arrives once on a transient_local (latched) topic, so without
  // this the stored reference would keep whatever cap was in force when the
  // single WaypointArray landed.
  void ingestWaypoints()
  {
    // The raceline's own speed profile is only a valid reference if the car
    // is actually allowed to drive it. Clamping to limits.speed_max keeps
    // vx_ref, r_ref = vx_ref*kappa, the horizon's arc-length advance and the
    // adaptive dt mutually consistent - see
    // ReferenceTrajectoryHandler::setSpeedLimit.
    ref_handler_.setSpeedLimit(get_parameter("limits.speed_max").as_double());
    ref_handler_.setWaypoints(last_waypoints_msg_);
    RCLCPP_INFO(
      get_logger(), "loaded %zu raceline waypoints at speed cap %.2f m/s",
      ref_handler_.waypointCount(), ref_handler_.speedLimit());
    if (ref_handler_.clampedWaypointCount() > 0) {
      RCLCPP_WARN(
        get_logger(),
        "raceline speed profile exceeds limits.speed_max=%.2f m/s at %zu/%zu waypoints "
        "(max %.2f m/s, %.1fx the limit) - clamped. The raceline was optimized for a faster "
        "vehicle than this one; regenerate it with v_max=%.2f m/s for a usable speed profile. "
        "Raising limits.speed_max only helps if the VEHICLE can really go that fast - the cap "
        "must describe the plant, not the ambition.",
        ref_handler_.speedLimit(), ref_handler_.clampedWaypointCount(),
        ref_handler_.waypointCount(), ref_handler_.maxRawSpeed(),
        ref_handler_.maxRawSpeed() / std::max(ref_handler_.speedLimit(), 1e-9),
        ref_handler_.speedLimit());
    }
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
    // Pick up a runtime change to limits.speed_max without needing the
    // raceline republished. Cheap scalar compare per cycle; the O(n) re-ingest
    // only runs on an actual change. Must come BEFORE the odom/waypoint
    // guards - re-clamping the stored reference does not depend on odometry,
    // and behind the guard a param change would be silently dropped whenever
    // odom happened to be absent.
    if (ref_handler_.hasWaypoints() &&
      get_parameter("limits.speed_max").as_double() != ref_handler_.speedLimit())
    {
      ingestWaypoints();
    }

    if (!has_odom_ || !ref_handler_.hasWaypoints()) {
      applyFallback(!has_odom_ ? "no odometry received yet" : "no raceline received yet");
      return;
    }

    constexpr double kStaleOdomTimeoutS = 0.5;
    if ((now() - last_odom_stamp_).seconds() > kStaleOdomTimeoutS) {
      applyFallback("stale odometry");
      return;
    }

    // Latency compensation. The command about to be computed will not reach
    // the tyres until the odom sample has aged by (sensing age + this solve +
    // one command-hold period), and the path-following loop gain grows as
    // v^2/L while that dead time stays constant - which is why a lag that is
    // harmless at 11 m/s destabilises the loop at 27 m/s. Roll the state
    // forward over that window with the last applied input so the MPC plans
    // from where the car will be, not where it was.
    const double control_period_s = 1.0 / std::max(param_manager_.controlRateHz(), 1.0);
    const double odom_age_s = std::clamp((now() - last_odom_stamp_).seconds(), 0.0, 0.2);
    const double horizon_s = std::clamp(
      odom_age_s + control_period_s + last_solve_time_ms_ * 1e-3, 0.0, 0.25);
    const State x0 = horizon_s > 0.0 ?
      controller_->predictState(current_state_, u_prev_, horizon_s) :
      current_state_;

    const MpcOutput out = controller_->computeCommand(x0, u_prev_, ref_handler_);
    if (!out.solved) {
      applyFallback("QP solve failed: " + out.status);
      return;
    }
    last_solve_time_ms_ = out.solve_time_ms;

    u_prev_ = out.u0;
    const double speed_min = get_parameter("limits.speed_min").as_double();
    const double speed_max = get_parameter("limits.speed_max").as_double();
    // Integrate from the predicted speed, consistent with the state the
    // command was actually computed for.
    const double speed_cmd = std::clamp(
      x0(3) + out.u0(1) * control_period_s, speed_min, speed_max);

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
    DebugPublisher::StatusInfo info;
    info.tire = controller_->effectiveTireParams();
    info.infeasible_ref_stages = out.infeasible_ref_stages;
    info.relinearized = out.relinearized;
    info.x0_prediction_s = horizon_s;
    debug_pub_->publishStatus(true, out.solve_time_ms, out.cost, "OK", info, now());
    if (out.infeasible_ref_stages > 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%d/%d horizon stages have no steady-state solution for the current tire params - the "
        "raceline asks for more lateral grip there than the model has. Tracking will be "
        "approximate on those stages.",
        out.infeasible_ref_stages, param_manager_.mpcConfig().N + 1);
    }

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
  // Last successful solve duration, used to size the latency-compensation
  // prediction in controlLoop().
  double last_solve_time_ms_{0.0};

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
