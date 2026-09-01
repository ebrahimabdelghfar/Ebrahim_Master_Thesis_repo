#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
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
    // Friction is a runtime property, so the reference's grip assumption has to
    // follow the identified tires rather than a constant tuned for one surface.
    // Re-ingesting cuts corner speeds to sqrt(a_lat_max/|kappa|) against the new
    // ceiling and re-smooths the profile, which is what keeps a steady-state
    // solution in existence at every stage when grip drops.
    identified_grip_ceiling_ = gripCeiling(tire);
    if (ref_handler_.hasWaypoints()) {
      ingestWaypoints();
    }
    response->ack = true;
  }

  // Peak lateral acceleration the identified tires can carry, both axles at
  // their Magic-Formula peak (Fy = D*Fz).
  double gripCeiling(const TireParams & tire) const
  {
    double fz_f = 0.0, fz_r = 0.0;
    vehicle_model_->normalLoads(fz_f, fz_r);
    return (std::abs(tire.Df) * fz_f + std::abs(tire.Dr) * fz_r) /
           vehicle_model_->vehicleParams().mass;
  }

  // What the reference is allowed to demand laterally: the identified ceiling
  // derated by limits.grip_utilization, never above the operator's
  // limits.lateral_accel_max cap. Before the first identification there is no
  // ceiling and the cap alone applies.
  double effectiveLateralLimit() const
  {
    const double cap = get_parameter("limits.lateral_accel_max").as_double();
    if (!(identified_grip_ceiling_ > 0.0)) {
      return cap;
    }
    const double util = get_parameter("limits.grip_utilization").as_double();
    return std::min(cap, identified_grip_ceiling_ * util);
  }

  // Which of the two limits is actually binding, for the ingest log.
  std::string gripSourceDescription() const
  {
    if (!(identified_grip_ceiling_ > 0.0)) {
      return "limits.lateral_accel_max, no tire identification yet";
    }
    char buf[160];
    const double util = get_parameter("limits.grip_utilization").as_double();
    std::snprintf(
      buf, sizeof(buf),
      "identified grip ceiling %.2f m/s^2 x %.2f utilization = %.2f, capped by "
      "limits.lateral_accel_max %.2f",
      identified_grip_ceiling_, util, identified_grip_ceiling_ * util,
      get_parameter("limits.lateral_accel_max").as_double());
    return buf;
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
    // A speed cap alone leaves every corner at whatever speed the raceline's
    // generator chose for its own (grippier) vehicle, so the reference never
    // asks this car to brake for the turn - see
    // ReferenceTrajectoryHandler::setLateralAccelLimit.
    ref_handler_.setLateralAccelLimit(effectiveLateralLimit());
    ref_handler_.setLongitudinalLimits(
      get_parameter("limits.decel_max").as_double(),
      get_parameter("limits.accel_max").as_double());
    ref_handler_.setWaypoints(last_waypoints_msg_);
    RCLCPP_INFO(
      get_logger(),
      "loaded %zu raceline waypoints at speed cap %.2f m/s, lateral cap %.2f m/s^2 (%.2f g, "
      "%s); reference now peaks at %.2f m/s^2 (%.2f g)",
      ref_handler_.waypointCount(), ref_handler_.speedLimit(),
      ref_handler_.lateralAccelLimit(), ref_handler_.lateralAccelLimit() / 9.81,
      gripSourceDescription().c_str(),
      ref_handler_.maxLateralDemand(), ref_handler_.maxLateralDemand() / 9.81);
    if (ref_handler_.curvatureClampedCount() > 0) {
      RCLCPP_WARN(
        get_logger(),
        "raceline demands up to %.2f m/s^2 (%.2f g) of lateral acceleration, above "
        "limits.lateral_accel_max=%.2f m/s^2 (%.2f g) at %zu/%zu waypoints - corner speeds "
        "cut to sqrt(a_lat_max/kappa) and the profile re-smoothed against decel_max/accel_max. "
        "The car will now brake for those corners, but it is driving a slower line than the "
        "raceline describes; regenerate the raceline for this vehicle's real grip.",
        ref_handler_.maxRawLateralDemand(), ref_handler_.maxRawLateralDemand() / 9.81,
        ref_handler_.lateralAccelLimit(), ref_handler_.lateralAccelLimit() / 9.81,
        ref_handler_.curvatureClampedCount(), ref_handler_.waypointCount());
    }
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

  // limits.drivetrain_tau_s is a property of whatever tracks our speed command
  // downstream, and getting it wrong is silent: the command leads the plan by
  // (control_period + tau), so a first-order speed loop of time constant
  // tau_plant delivers only (control_period + tau)/tau_plant of the planned
  // acceleration. At tau = 0 against a 0.3 s plant that is 6.7 %, which reads
  // as "the car does not brake for corners".
  //
  // Treating the measured response as that same first-order lag,
  // tau_plant = (v_cmd - v)/vdot. With limits.drivetrain_tau_auto (default
  // true) the estimate is written straight back into limits.drivetrain_tau_s
  // once it has settled, so the compensation is self-tuning: the parameter is
  // a plant property the node can measure, and leaving it at a hand-entered
  // 0.0 is the worst case in every direction (offline: too low costs 3.42 m/s
  // of corner overspeed, too high by 3x costs 1.13).
  //
  // `driving` MUST be false whenever speed_cmd is not actually published -
  // in managed mode before the handover another controller is driving, and
  // the speed response then has nothing to do with our command.
  //
  // Accelerating and braking are identified SEPARATELY, because a speed loop
  // that opens a throttle one way and releases it (plus brakes) the other is
  // not one first-order lag. Sharing a constant averages the two, and the
  // acceleration phase supplies most of the samples on a raceline - so the
  // braking compensation comes out biased toward the faster half and the car
  // enters the corner having delivered a fraction of the planned deceleration.
  // Measured offline over 250 s on traj_race_cl.csv (plant speed loop written
  // as tau_accel/tau_decel, plant peak axle mu 0.9, tau identified online),
  // spin events on |beta| > 20 deg and peak achieved lateral acceleration:
  //   plant 0.5/5.0   shared -> 11 spins, 63.3 m/s^2   split -> 0, 11.5
  //   plant 2.77/5.0  shared -> 13 spins, 41.7 m/s^2   split -> 0, 12.8
  //   plant 0.5/2.77  shared -> 11 spins, 59.4 m/s^2   split -> 0,  7.8
  // On a symmetric plant the two estimates coincide and nothing changes.
  void estimateDrivetrainTau(
    double speed_meas, double speed_cmd, double configured_tau,
    double configured_tau_decel, double control_period_s, bool driving)
  {
    if (!driving) {
      has_prev_speed_sample_ = false;
      return;
    }
    if (has_prev_speed_sample_ && control_period_s > 0.0) {
      const double drive = prev_speed_cmd_ - prev_speed_meas_;
      const double vdot = (speed_meas - prev_speed_meas_) / control_period_s;
      // Only informative while the loop is actually being driven and is
      // responding in the commanded direction.
      if (std::abs(drive) > 0.5 && drive * vdot > 0.0 && std::abs(vdot) > 1e-3) {
        const double tau_sample = std::clamp(drive / vdot, 0.0, kDrivetrainTauMax);
        if (std::isfinite(tau_sample) && tau_sample > 0.0) {
          const bool braking = drive < 0.0;
          double & estimate = braking ? tau_estimate_decel_ : tau_estimate_;
          int & samples = braking ? tau_samples_decel_ : tau_samples_;
          estimate = (samples > 0) ? 0.98 * estimate + 0.02 * tau_sample : tau_sample;
          ++samples;
          applyDrivetrainTau(
            braking ? "limits.drivetrain_tau_decel_s" : "limits.drivetrain_tau_s",
            braking ? configured_tau_decel : configured_tau, estimate, samples,
            control_period_s);
        }
      }
    }
    prev_speed_cmd_ = speed_cmd;
    prev_speed_meas_ = speed_meas;
    has_prev_speed_sample_ = true;
  }

  // Writes the settled estimate back into limits.drivetrain_tau_s, or reports
  // it when limits.drivetrain_tau_auto is off. Held until the EMA has enough
  // samples to be a time constant rather than one noisy difference, and
  // rewritten only on a change worth acting on so the parameter does not
  // dither under the speed command it is itself shaping.
  void applyDrivetrainTau(
    const char * param, double configured_tau, double estimate, int samples,
    double control_period_s)
  {
    if (samples < kDrivetrainTauMinSamples) {
      return;
    }
    if (!get_parameter("limits.drivetrain_tau_auto").as_bool()) {
      if (std::abs(estimate - configured_tau) > 0.05) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "drivetrain speed loop looks like a first-order lag of tau=%.2f s but "
          "%s is %.2f s - the plant is receiving about %.0f%% of "
          "the planned acceleration. Set %s to %.2f, or turn on "
          "limits.drivetrain_tau_auto.",
          estimate, param, configured_tau,
          100.0 * (control_period_s + configured_tau) / std::max(estimate, 1e-6),
          param, estimate);
      }
      return;
    }
    if (std::abs(estimate - configured_tau) <= kDrivetrainTauDeadband) {
      return;
    }
    set_parameter(rclcpp::Parameter(param, estimate));
    // RCLCPP_INFO(
    //   get_logger(),
    //   "%s %.2f -> %.2f s from %d measured samples of the speed loop "
    //   "(was delivering about %.0f%% of the planned acceleration)",
    //   param, configured_tau, estimate, samples,
    //   100.0 * (control_period_s + configured_tau) / std::max(estimate, 1e-6));
  }

  // Same command, fresh stamp: the plan is still the one computed for the
  // most recent state, and downstream consumers time out on the header.
  void republishLastCommand()
  {
    if (!has_last_command_) {
      return;
    }
    ackermann_msgs::msg::AckermannDriveStamped cmd = last_command_;
    cmd.header.stamp = now();
    if (standalone_mode_ || start_working_) {
      drive_pub_->publish(cmd);
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
    // Clear the predicted path. Without this the last successful horizon stays
    // on the topic while the car drives on under hold_last, so it renders
    // further and further behind the vehicle and reads as a bad solve.
    debug_pub_->publishPredictedPath({}, "map", now());
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
    // Re-solving for an odom sample already solved for produces a DIFFERENT
    // command every cycle - horizon_s grows with the sample's age, so the
    // state is extrapolated further and the QP answers a question built from
    // no new information. The model then steers against its own extrapolation
    // until the next real sample snaps the state back, which is a self-excited
    // ripple at the beat between the two rates.
    //
    // It bites here because the CARLA bridge publishes /odom at the SERVER
    // rate (odometry.follow_server_rate, so 1/sim.fixed_delta_seconds = 30 Hz)
    // while control_rate_hz is 50. Measured over 90 s on traj_race_cl.csv,
    // odom 30 Hz against control 50 Hz: 848 steering sign reversals, RMS
    // steering rate 0.349 rad/s and the rate limit binding on 15 cycles at
    // N=50 - and 1987 reversals with the limit binding 1248 times at N=100.
    // Holding the last command until new information arrives: 2 and 6
    // reversals, RMS 0.043 rad/s, no saturation, tracking unchanged.
    if (param_manager_.solveOnNewOdomOnly() && has_solved_once_ &&
      last_odom_stamp_ == last_solved_odom_stamp_)
    {
      republishLastCommand();
      return;
    }
    last_solved_odom_stamp_ = last_odom_stamp_;

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

    // Rate saturation on the first steering step means steering_rate_max is
    // the binding constraint, not the tracking cost - worth knowing while it
    // is still an unmeasured placeholder.
    const double steer_rate_max = get_parameter("limits.steering_rate_max").as_double();
    const double steer_step = std::abs(out.u0(0) - u_prev_(0));
    if (out.dt_used > 0.0 && steer_step >= 0.99 * steer_rate_max * out.dt_used) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "steering rate saturated: %.4f rad in %.3f s = %.2f rad/s at the "
        "limits.steering_rate_max cap of %.2f rad/s",
        steer_step, out.dt_used, steer_step / out.dt_used, steer_rate_max);
    }

    u_prev_ = out.u0;
    const double speed_min = get_parameter("limits.speed_min").as_double();
    const double speed_max = get_parameter("limits.speed_max").as_double();
    // Integrate from the predicted speed, consistent with the state the
    // command was actually computed for. The extra tau term inverts the
    // first-order lag of whatever tracks this speed command downstream: to
    // make the plant follow the planned speed, ask for where the plan will be
    // one lag constant later. tau = 0 reduces to one control period of accel.
    // Braking uses its own constant: the loop is slower on the brakes than on
    // the throttle, and compensating a 5 s braking lag with a 3 s number is
    // what lets the car arrive at the apex still carrying entry speed.
    const double drivetrain_tau_s = get_parameter("limits.drivetrain_tau_s").as_double();
    const double tau_decel_param = get_parameter("limits.drivetrain_tau_decel_s").as_double();
    const double drivetrain_tau_decel_s =
      tau_decel_param >= 0.0 ? tau_decel_param : drivetrain_tau_s;
    const double tau_used = out.u0(1) < 0.0 ? drivetrain_tau_decel_s : drivetrain_tau_s;
    const double speed_cmd = std::clamp(
      x0(3) + out.u0(1) * (control_period_s + tau_used), speed_min, speed_max);
    estimateDrivetrainTau(
      current_state_(3), speed_cmd, drivetrain_tau_s, drivetrain_tau_decel_s,
      control_period_s, standalone_mode_ || start_working_);

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
    has_solved_once_ = true;

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
  // Peak lateral acceleration of the last ACCEPTED identified tire set; 0 until
  // the first mpc/update_params, when limits.lateral_accel_max alone applies.
  double identified_grip_ceiling_{0.0};
  double prev_speed_cmd_{0.0};
  double prev_speed_meas_{0.0};
  double tau_estimate_{0.0};
  double tau_estimate_decel_{0.0};
  int tau_samples_{0};
  int tau_samples_decel_{0};
  bool has_prev_speed_sample_{false};
  // Enough samples for the EMA to be a time constant rather than one noisy
  // difference; a deadband so the parameter does not dither under the command
  // it is itself shaping; a ceiling so a near-zero vdot cannot produce one.
  static constexpr int kDrivetrainTauMinSamples = 50;
  static constexpr double kDrivetrainTauDeadband = 0.02;
  static constexpr double kDrivetrainTauMax = 5.0;
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
  rclcpp::Time last_solved_odom_stamp_;
  bool has_solved_once_{false};
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
