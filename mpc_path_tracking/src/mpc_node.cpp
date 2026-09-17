#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "adaptive_controller_interfaces/msg/friction_estimate.hpp"
#include "adaptive_controller_interfaces/srv/identified_param.hpp"
#include "f1tenth_msgs/msg/waypoint_array.hpp"
#include "mpc_path_tracking/debug_publisher.hpp"
#include "mpc_path_tracking/grip_limits.hpp"
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
    // limits.grip_utilization was measured against THIS tire, so the startup set
    // is the reference the identified tire's shape is compared with. See
    // grip::shapeUtilization.
    reference_tire_ = tire_params;
    // The startup tire IS the reference, so its shape factor is unity and the
    // margin a fast-mu-only ceiling gets is the configured base.
    util_shape_ = get_parameter("limits.grip_utilization").as_double();

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

    // Latched to match On-Track-SysID's publisher, so a late-starting node still
    // gets the most recent estimate. Nothing publishing here is a supported
    // configuration - see frictionCallback() and effectiveGripBudget().
    rclcpp::QoS friction_qos(1);
    friction_qos.transient_local();
    friction_sub_ = create_subscription<adaptive_controller_interfaces::msg::FrictionEstimate>(
      get_parameter("mu_fast.topic").as_string(), friction_qos,
      std::bind(&MpcNode::frictionCallback, this, _1));

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
    // Above the oversteer critical speed every horizon stage linearized there
    // diverges, and over N stages that is rho^N in the condensed QP - which the
    // solver can only report as a generic failure. Capping the reference speed
    // below v_crit keeps every stage linearization stable, so a low-grip set is
    // usable rather than refused. Under a decaying surface that distinction is
    // the whole game: v_crit falls with grip, and rejecting on it would freeze
    // the controller on the last grippy model for the rest of the run.
    const double v_crit = grip::criticalSpeed(tire, vehicle_model_->vehicleParams());
    const double v_cap = get_parameter("limits.critical_speed_safety").as_double() * v_crit;
    const double v_floor = get_parameter("limits.critical_speed_floor").as_double();
    if (v_cap < v_floor) {
      RCLCPP_ERROR(
        get_logger(),
        "mpc/update_params REJECTED: identified tire set is oversteering with critical speed "
        "%.1f m/s, capping the reference at %.1f m/s - below limits.critical_speed_floor=%.1f "
        "m/s, which describes a car that cannot drive this track at all. Keeping the previous "
        "tire params.",
        v_crit, v_cap, v_floor);
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
    pacejka_grip_ceiling_ = grip::gripCeiling(tire, vehicle_model_->vehicleParams());
    // Braking is the case that binds, because that is the state the car is in at a
    // trail-braked apex. Carried as a ratio so the rate limiter and the sigma
    // tightening keep acting on one scalar - see effectiveGripBudget.
    axle_lateral_ratio_ = pacejka_grip_ceiling_ > 0.0
      ? grip::axleGripCeiling(
      tire, vehicle_model_->vehicleParams(),
      -get_parameter("limits.decel_max").as_double()) / pacejka_grip_ceiling_
      : 1.0;
    identified_speed_cap_ = v_cap;
    // The fast mu derates RELATIVE to the surface the coefficient set was fitted
    // on, so re-anchoring here is what stops the same friction drop being
    // counted twice - once in D, once in the ratio.
    mu_anchor_ = fast_mu_valid_ ? fast_mu_ : 0.0;
    fast_ratio_ = 1.0;
    applyGripTarget(pacejka_grip_ceiling_);
    util_shape_ = grip::shapeUtilization(
      tire, reference_tire_, get_parameter("limits.grip_utilization").as_double());
    if (ref_handler_.hasWaypoints()) {
      ingestWaypoints();
    }
    response->ack = true;
  }

  // Optional fast derate on top of the identified D, published by On-Track-SysID
  // far more often than a full coefficient set is fitted. An invalid tick is
  // normal (the brush gate needs slip the car does not generate on a straight),
  // so it holds the last ratio rather than collapsing the adaptation.
  void frictionCallback(
    const adaptive_controller_interfaces::msg::FrictionEstimate::SharedPtr msg)
  {
    const rclcpp::Time now_stamp = now();
    if (!msg->valid || !(msg->mu > 0.0)) {
      return;
    }
    if (!fast_mu_valid_) {
      RCLCPP_INFO(
        get_logger(), "fast friction estimate acquired on %s: grip now follows mu as well as D",
        get_parameter("mu_fast.topic").as_string().c_str());
    }
    fast_mu_ = msg->mu;
    fast_sigma_mu_ = msg->sigma_mu;
    fast_mu_valid_ = true;
    last_fast_stamp_ = now_stamp;

    // With no coefficient set yet there is nothing to derate relative to, and the
    // startup tire block is an assumption about the surface rather than a
    // measurement of it. The fast mu is a measurement, so it stands in as D until
    // On-Track-SysID fits one. Leaving mu_anchor_ unset keeps the first
    // coefficient set free to anchor the ratio where it belongs.
    if (!(pacejka_grip_ceiling_ > 0.0)) {
      setGripFromIsotropicMu(fast_mu_);
      return;
    }

    // The first estimate after a coefficient set arrives defines the anchor, so
    // the ratio starts at 1.0 and drifts only as the surface does.
    if (!(mu_anchor_ > 0.0)) {
      mu_anchor_ = fast_mu_;
    }
    const auto bounds = get_parameter("mu_fast.ratio_bounds").as_double_array();
    const double lo = bounds.size() == 2 ? bounds[0] : 0.4;
    const double hi = bounds.size() == 2 ? bounds[1] : 1.2;
    fast_ratio_ = std::clamp(fast_mu_ / mu_anchor_, lo, hi);
    applyGripTarget(pacejka_grip_ceiling_ * fast_ratio_);
  }

  // Grip from a friction estimate alone. An isotropic mu is a Magic Formula peak
  // of D = mu on both axles, so the ceiling and the binding axle come out of the
  // same arithmetic a coefficient set goes through.
  void setGripFromIsotropicMu(double mu)
  {
    TireParams isotropic = reference_tire_;
    isotropic.Df = mu;
    isotropic.Dr = mu;
    const VehicleParams & vp = vehicle_model_->vehicleParams();
    const double ceiling = grip::gripCeiling(isotropic, vp);
    axle_lateral_ratio_ = ceiling > 0.0
      ? grip::axleGripCeiling(
      isotropic, vp, -get_parameter("limits.decel_max").as_double()) / ceiling
      : 1.0;
    applyGripTarget(ceiling);
  }

  // Moves the ceiling toward `target`, instantly downward and on a ramp upward,
  // then recuts the reference against it. The ramp is timed here rather than by
  // the caller so that every route to the ceiling is rate limited, a coefficient
  // set included: a fit that claims twice the grip is still only a claim.
  void applyGripTarget(double target)
  {
    if (!(target > 0.0)) {
      return;
    }
    const rclcpp::Time stamp = now();
    const double dt = identified_grip_ceiling_ > 0.0 ? (stamp - last_ceiling_stamp_).seconds() : 0.0;
    last_ceiling_stamp_ = stamp;
    identified_grip_ceiling_ = grip::rateLimitedCeiling(
      identified_grip_ceiling_, target,
      get_parameter("limits.grip_rise_rate_per_s").as_double(), dt);
    if (ref_handler_.hasWaypoints()) {
      ingestWaypoints();
    }
  }

  // True once a fast estimate has arrived and has not since gone stale. Losing
  // the publisher entirely is a supported configuration, not a fault: the arms
  // that run without a friction warm start never publish at all.
  bool fastEstimateFresh()
  {
    if (!fast_mu_valid_) {
      return false;
    }
    const double age = (now() - last_fast_stamp_).seconds();
    if (age <= get_parameter("mu_fast.timeout_s").as_double()) {
      return true;
    }
    RCLCPP_WARN(
      get_logger(),
      "fast friction estimate stale by %.1f s - reverting the grip ceiling to the identified "
      "Pacejka D alone. The recovery still ramps at limits.grip_rise_rate_per_s.",
      age);
    fast_mu_valid_ = false;
    fast_ratio_ = 1.0;
    return false;
  }

  // What the reference may demand of the tires in any direction: the grip
  // ceiling times the utilization margin. Before the first identification there
  // is no ceiling and the operator's caps alone apply.
  double effectiveGripBudget()
  {
    if (!(identified_grip_ceiling_ > 0.0)) {
      return std::numeric_limits<double>::infinity();
    }
    double util = util_shape_;
    if (fastEstimateFresh()) {
      util = grip::applySigmaTightening(
        util, fast_mu_, fast_sigma_mu_,
        get_parameter("limits.sigma_tighten_gain").as_double(),
        get_parameter("limits.grip_utilization_min").as_double());
    }
    return identified_grip_ceiling_ * util;
  }

  // Which limit is actually binding, and where the grip figure came from, for
  // the ingest log.
  std::string gripSourceDescription()
  {
    if (!(identified_grip_ceiling_ > 0.0)) {
      return "limits.lateral_accel_max, no tire identification yet";
    }
    char buf[256];
    std::snprintf(
      buf, sizeof(buf),
      "grip ceiling %.2f m/s^2 (Pacejka D %.2f x fast mu ratio %.2f, %s) x %.2f utilization "
      "(base %.2f, shape %.2f) x %.2f binding axle under braking = %.2f, capped by "
      "limits.lateral_accel_max %.2f",
      identified_grip_ceiling_, pacejka_grip_ceiling_, fast_ratio_,
      fastEstimateFresh() ? "fast mu live" : "D only",
      effectiveGripBudget() / identified_grip_ceiling_,
      get_parameter("limits.grip_utilization").as_double(), util_shape_, axle_lateral_ratio_,
      effectiveGripBudget() * axle_lateral_ratio_,
      get_parameter("limits.lateral_accel_max").as_double());
    return buf;
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
    // Also capped below the oversteer critical speed of the identified tires, so
    // no stage of the horizon is linearized where the model diverges.
    ref_handler_.setSpeedLimit(
      std::min(get_parameter("limits.speed_max").as_double(), identified_speed_cap_));
    // A speed cap alone leaves every corner at whatever speed the raceline's
    // generator chose for its own (grippier) vehicle, so the reference never
    // asks this car to brake for the turn - see
    // ReferenceTrajectoryHandler::setLateralAccelLimit.
    // Only the lateral cap takes the axle ratio. Straight-line braking loads the
    // axle it takes the load from, so the longitudinal budget below is unaffected.
    const double a_grip = effectiveGripBudget();
    ref_handler_.setLateralAccelLimit(
      std::min(
        get_parameter("limits.lateral_accel_max").as_double(), a_grip * axle_lateral_ratio_));
    // The same budget bounds braking and driving. Leaving these at their yaml
    // values means the backward sweep keeps believing in a braking authority the
    // surface no longer supports, and the reference then arrives at the apex too
    // fast however correct its corner speed was.
    ref_handler_.setFrictionEllipse(get_parameter("limits.friction_ellipse").as_bool());
    ref_handler_.setLongitudinalLimits(
      std::min(get_parameter("limits.decel_max").as_double(), a_grip),
      std::min(get_parameter("limits.accel_max").as_double(), a_grip));
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
  //
  // Three things keep the identification from measuring itself. Offline over
  // 250 s on traj_race_cl.csv against a plant of true lag 2.77 s with 0.10 m/s
  // of speed noise (`scratchpad/corner.cpp`), settled tau / parameter rewrites
  // / mean cycle-to-cycle command step:
  //   single tick, EMA of ratios   3.38   1642   6.16 km/h
  //   1 s window + least squares   2.90      7   3.44 km/h
  // and against a plant of true lag 0.5 s, 1.12 -> 0.60 s.
  //
  // 1. A 2.77 s lag moves the speed by under 2% in one 0.05 s control period,
  //    so a single-tick difference is mostly odometry noise. The window is
  //    kDrivetrainTauWindowS of command error against the speed change over
  //    the same span.
  // 2. tau enters as drive/vdot, and averaging that ratio lets the samples with
  //    the smallest (noisiest) vdot dominate. Forgetting least squares on
  //    vdot = drive/tau weights each sample by drive^2 instead.
  // 3. A torque-limited plant clips vdot while the tau term keeps pushing the
  //    command further ahead - drive/vdot then measures the limiter, and the
  //    result feeds straight back into the command. The published speed is
  //    therefore clamped to what the drivetrain has been seen to reach within
  //    one lag constant, and no sample is taken while that clamp is active.
  //    Against the same plant torque-limited to 1.0 m/s^2, tau ran to the 5 s
  //    ceiling without this and settles at 3.46 s with it.
  void estimateDrivetrainTau(
    double speed_meas, double speed_cmd, double configured_tau,
    double configured_tau_decel, double control_period_s, bool driving)
  {
    if (!driving) {
      speed_cmd_history_.clear();
      speed_meas_history_.clear();
      return;
    }
    if (control_period_s <= 0.0) {
      return;
    }
    speed_cmd_history_.push_back(speed_cmd);
    speed_meas_history_.push_back(speed_meas);
    const size_t win = std::max<size_t>(
      1, static_cast<size_t>(std::lround(kDrivetrainTauWindowS / control_period_s)));
    if (speed_meas_history_.size() > win + 1) {
      speed_cmd_history_.erase(speed_cmd_history_.begin());
      speed_meas_history_.erase(speed_meas_history_.begin());
    }
    if (clamp_hold_cycles_ > 0) {
      --clamp_hold_cycles_;
    }
    if (speed_meas_history_.size() <= win) {
      return;
    }
    const size_t n = speed_meas_history_.size();
    double sum_drive = 0.0;
    for (size_t i = n - 1 - win; i + 1 < n; ++i) {
      sum_drive += speed_cmd_history_[i] - speed_meas_history_[i];
    }
    const double drive = sum_drive / static_cast<double>(win);
    const double span_s = static_cast<double>(win) * control_period_s;
    const double vdot = (speed_meas_history_[n - 1] - speed_meas_history_[n - 1 - win]) / span_s;
    updateAccelCeiling(vdot);
    // Only informative while the loop is actually being driven, is responding
    // in the commanded direction and is not against the command clamp.
    if (std::abs(drive) <= 0.5 || drive * vdot <= 0.0 || std::abs(vdot) <= 1e-3 ||
      clamp_hold_cycles_ > 0)
    {
      return;
    }
    const bool braking = drive < 0.0;
    double & s_dd = braking ? tau_ls_dd_decel_ : tau_ls_dd_;
    double & s_dv = braking ? tau_ls_dv_decel_ : tau_ls_dv_;
    s_dd = kDrivetrainTauForgetting * s_dd + drive * drive;
    s_dv = kDrivetrainTauForgetting * s_dv + drive * vdot;
    if (!(std::abs(s_dv) > 1e-9)) {
      return;
    }
    const double estimate = std::clamp(s_dd / s_dv, 0.0, kDrivetrainTauMax);
    if (!std::isfinite(estimate) || estimate <= 0.0) {
      return;
    }
    double & held = braking ? tau_estimate_decel_ : tau_estimate_;
    int & samples = braking ? tau_samples_decel_ : tau_samples_;
    held = estimate;
    ++samples;
    applyDrivetrainTau(
      braking ? "limits.drivetrain_tau_decel_s" : "limits.drivetrain_tau_s",
      braking ? configured_tau_decel : configured_tau, held, samples,
      control_period_s);
  }

  // Highest acceleration the drivetrain has actually delivered lately, per
  // direction, floored at the planner's own limit so the decay only ever walks
  // back a ceiling the plant earned above it. Decaying below that limit
  // deadlocks limitUnreachableSpeed: a command capped at `ceiling * reach_s`
  // delivers `ceiling` again, so a ceiling shrunk over a straight cannot be
  // earned back and the car reaches the next apex still unable to brake.
  void updateAccelCeiling(double vdot)
  {
    const bool braking = vdot <= 0.0;
    double & ceiling = braking ? decel_ceiling_ : accel_ceiling_;
    const double configured =
      get_parameter(braking ? "limits.decel_max" : "limits.accel_max").as_double();
    ceiling = std::max(configured, std::max(kAccelCeilingDecay * ceiling, std::abs(vdot)));
  }

  // Anti-windup: a speed the drivetrain cannot reach within one lag constant
  // is not a setpoint, and the error it leaves is what corrupts the lag
  // identification. Returns the command to publish and records whether the
  // limit bound.
  double limitUnreachableSpeed(double speed_cmd, double speed_now, double reach_s)
  {
    const double limited = std::clamp(
      speed_cmd, speed_now - decel_ceiling_ * reach_s, speed_now + accel_ceiling_ * reach_s);
    if (std::abs(limited - speed_cmd) > 1e-9) {
      clamp_hold_cycles_ = kDrivetrainTauClampHoldCycles;
    }
    return limited;
  }

  // Writes the settled estimate back into limits.drivetrain_tau_s, or reports
  // it when limits.drivetrain_tau_auto is off. Held until the fit has enough
  // samples to be a time constant rather than one noisy window, and rewritten
  // only on a change worth acting on so the parameter does not dither under
  // the speed command it is itself shaping. The rewrite is also rate limited
  // in both size and frequency: every rewrite moves the published speed by the
  // planned acceleration times the change, so a jumpy parameter is a jumpy
  // command (measured offline: 1642 rewrites and 6.16 km/h of mean
  // cycle-to-cycle command step, against 7 and 3.44 with the limits on).
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
    const rclcpp::Time stamp = now();
    double & last_rewrite = std::strstr(param, "decel") ? last_tau_rewrite_decel_ : last_tau_rewrite_;
    if (last_rewrite > 0.0 && stamp.seconds() - last_rewrite < kDrivetrainTauMinRewriteS) {
      return;
    }
    const double written = std::clamp(
      estimate, configured_tau / kDrivetrainTauMaxRatio, configured_tau * kDrivetrainTauMaxRatio);
    last_rewrite = stamp.seconds();
    set_parameter(rclcpp::Parameter(param, written));
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
    if (accel_ceiling_ <= 0.0) {
      accel_ceiling_ = get_parameter("limits.accel_max").as_double();
      decel_ceiling_ = get_parameter("limits.decel_max").as_double();
    }
    const double speed_cmd = std::clamp(
      limitUnreachableSpeed(
        x0(3) + out.u0(1) * (control_period_s + tau_used), x0(3),
        control_period_s + tau_used),
      speed_min, speed_max);
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
  // Peak lateral acceleration the reference is planned against; 0 until the
  // first fast mu or mpc/update_params, when limits.lateral_accel_max alone
  // applies. Equal to pacejka_grip_ceiling_ until a fast mu estimate derates it,
  // and moved by grip::rateLimitedCeiling rather than assigned.
  double identified_grip_ceiling_{0.0};
  // Peak lateral acceleration of the last ACCEPTED coefficient set on its own.
  double pacejka_grip_ceiling_{0.0};
  // The tire limits.grip_utilization was tuned against, fixed at startup.
  TireParams reference_tire_;
  double util_shape_{1.0};
  // Fraction of the ceiling the binding axle can carry while braking at
  // limits.decel_max. Unity until a fast mu or a coefficient set arrives.
  double axle_lateral_ratio_{1.0};
  double identified_speed_cap_{std::numeric_limits<double>::infinity()};
  // Peak friction at the moment the last coefficient set was fitted, so the fast
  // estimate derates relative to that surface rather than absolutely.
  double mu_anchor_{0.0};
  double fast_mu_{0.0};
  double fast_sigma_mu_{0.0};
  double fast_ratio_{1.0};
  bool fast_mu_valid_{false};
  rclcpp::Time last_fast_stamp_;
  rclcpp::Time last_ceiling_stamp_;
  std::vector<double> speed_cmd_history_;
  std::vector<double> speed_meas_history_;
  double tau_estimate_{0.0};
  double tau_estimate_decel_{0.0};
  double tau_ls_dd_{0.0};
  double tau_ls_dv_{0.0};
  double tau_ls_dd_decel_{0.0};
  double tau_ls_dv_decel_{0.0};
  int tau_samples_{0};
  int tau_samples_decel_{0};
  double accel_ceiling_{0.0};
  double decel_ceiling_{0.0};
  int clamp_hold_cycles_{0};
  double last_tau_rewrite_{0.0};
  double last_tau_rewrite_decel_{0.0};
  // Enough samples for the fit to be a time constant rather than one noisy
  // window; a deadband so the parameter does not dither under the command it
  // is itself shaping; a ceiling so a near-zero vdot cannot produce one.
  static constexpr int kDrivetrainTauMinSamples = 50;
  static constexpr double kDrivetrainTauDeadband = 0.02;
  static constexpr double kDrivetrainTauMax = 5.0;
  // Window long enough to move the speed well clear of the odometry noise,
  // short enough to stay inside one straight.
  static constexpr double kDrivetrainTauWindowS = 1.0;
  static constexpr double kDrivetrainTauForgetting = 0.99;
  // A drivetrain lag does not triple in a second, and one rewrite per second
  // is as fast as a 2.77 s constant can carry information.
  static constexpr double kDrivetrainTauMaxRatio = 1.2;
  static constexpr double kDrivetrainTauMinRewriteS = 1.0;
  static constexpr double kAccelCeilingDecay = 0.995;
  static constexpr int kDrivetrainTauClampHoldCycles = 20;
  std::unique_ptr<DebugPublisher> debug_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f1tenth_msgs::msg::WaypointArray>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<adaptive_controller_interfaces::msg::FrictionEstimate>::SharedPtr
    friction_sub_;
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
