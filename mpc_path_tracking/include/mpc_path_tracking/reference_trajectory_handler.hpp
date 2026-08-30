#ifndef MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_
#define MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "f1tenth_msgs/msg/waypoint_array.hpp"

namespace mpc_path_tracking
{

struct ReferencePoint
{
  double s{0.0};
  double x{0.0};
  double y{0.0};
  double psi{0.0};
  double kappa{0.0};
  double vx{0.0};
  double ax{0.0};
};

// Buffers the latched raceline (f1tenth_msgs/WaypointArray) and extracts,
// each control cycle, a forward-looking reference window used to build the
// LTV-MPC's time-varying cost/linearization sequence.
class ReferenceTrajectoryHandler
{
public:
  void setWaypoints(const f1tenth_msgs::msg::WaypointArray & msg);
  bool hasWaypoints() const { return !waypoints_.empty(); }
  size_t waypointCount() const { return waypoints_.size(); }

  // Upper bound applied to every waypoint's vx_mps at ingest, normally
  // limits.speed_max. A raceline whose speed profile was optimized for a
  // faster car than the one running it makes EVERY downstream reference
  // quantity infeasible at once: the tracked speed target vx_ref, the yaw
  // rate target r_ref = vx_ref * kappa, the arc-length advance
  // s_{k+1} = s_k + vx_ref*dt that places the horizon, and the adaptive dt.
  // The horizon then anchors itself vx_ref/speed_max times further down the
  // track than the car can reach, so each stage's lateral-error frame
  // belongs to a piece of track the car is not on, and the resulting
  // command weaves. Clamping at ingest keeps all four consistent.
  // Must be called before setWaypoints() to affect the current raceline.
  void setSpeedLimit(double speed_max) { speed_limit_ = speed_max; }
  double speedLimit() const { return speed_limit_; }

  // Peak lateral acceleration the reference is allowed to ask for, in m/s^2.
  // speed_max alone cannot make a raceline drivable: the CSV's speed profile
  // was optimized for whatever grip its generator assumed, so a corner can
  // sit far below the speed cap and still demand more than the tires can
  // give. Nothing downstream re-derives the corner speed, so the car simply
  // arrives too fast, saturates an axle and runs wide - which is what the
  // "doesn't slow down for corners" symptom actually is. Clamping vx to
  // sqrt(a_lat_max/|kappa|) at ingest is what makes the reference itself
  // decelerate into the turn.
  //
  // MEASURED (offline closed loop, traj_race_cl.csv, controller model equal
  // to the plant, peak axle mu = 1.0 i.e. a 9.81 m/s^2 ceiling): the usable
  // budget is a cliff, not a gradient - a peak demand of 7.84 m/s^2 (0.80 g)
  // tracks to max|e_y| 0.29 m, 8.48 (0.86 g) to 24.0 m, 9.88 (1.01 g) to
  // 34.0 m. A Pacejka axle only reaches its peak at its peak slip angle, so
  // the closed loop needs the margin. Budget ~0.7 * the grip ceiling.
  // Must be called before setWaypoints() to affect the current raceline.
  void setLateralAccelLimit(double a_lat_max) { lateral_accel_limit_ = a_lat_max; }
  double lateralAccelLimit() const { return lateral_accel_limit_; }

  // Longitudinal limits used to make the curvature-clamped profile reachable:
  // a corner speed the car cannot brake down to in the distance available is
  // still an infeasible reference. Applied as backward (decel) and forward
  // (accel) passes around the closed track. Non-positive values disable the
  // corresponding pass. Must be called before setWaypoints().
  void setLongitudinalLimits(double decel_max, double accel_max)
  {
    decel_limit_ = decel_max;
    accel_limit_ = accel_max;
  }

  // Number of waypoints whose vx_mps exceeded speed_limit_ and were clamped
  // by the last setWaypoints() call, and the largest speed seen. Non-zero
  // means the raceline does not match the vehicle it is being driven on.
  size_t clampedWaypointCount() const { return clamped_count_; }
  double maxRawSpeed() const { return max_raw_speed_; }

  // Waypoints whose speed was cut by the curvature limit rather than by
  // speed_max, and the peak lateral demand of the raw (unclamped) profile.
  // Non-zero means the raceline was optimized for more grip than this car is
  // configured to have.
  size_t curvatureClampedCount() const { return curvature_clamped_count_; }
  double maxRawLateralDemand() const { return max_raw_lateral_demand_; }

  // Peak lateral acceleration the (speed-clamped) raceline demands,
  // max(vx^2 * |kappa|) [m/s^2]. A vehicle model whose tires cannot produce
  // this much cannot represent the trajectory it is being asked to track:
  // most horizon stages then have no steady-state solution and the MPC plans
  // against a model that believes the corner is impossible.
  double maxLateralDemand() const { return max_lateral_demand_; }

  // Nearest reference point (by Euclidean distance) to (x, y). Uses
  // last_nearest_index_ as a search hint and wraps around for closed
  // (looped) tracks, so cost stays near O(1) per call in steady tracking.
  ReferencePoint nearestPoint(double x, double y) const;

  // Builds N+1 reference points (k = 0..N) starting at the projection of
  // (x, y) onto the raceline, spaced by accumulated arc length, wrapping
  // around the (closed) track length. psi is unwrapped to stay continuous
  // with `psi_hint` (the vehicle's current heading) so downstream
  // heading-error terms never see a 2*pi jump.
  //
  // `v0` is the vehicle's current speed. The horizon is walked forward at a
  // speed that STARTS there and ramps toward each stage's vx_ref at
  // accel_max / decel_max, not at vx_ref itself. Walking at vx_ref assumes
  // the car is already doing the reference speed; when it is not, stage k
  // lands somewhere the car cannot be at k*dt, so every stage's lateral-error
  // frame belongs to a piece of track the car is not on and the solution
  // curves off the line. Measured on traj_race_cl.csv with the speed loop
  // uncompensated: the reference window reached 2.16x as far as the car
  // could travel and the published horizon diverged 11.7 m from a nonlinear
  // rollout of its own inputs.
  //
  // Pass v0 < 0 to walk at vx_ref (the old behaviour).
  std::vector<ReferencePoint> buildHorizon(
    double x, double y, double psi_hint, int horizon_steps, double dt,
    double v0 = -1.0, double accel_max = 0.0, double decel_max = 0.0) const;

  // Arc length of the perpendicular projection of (x, y) onto the raceline
  // polyline. Unlike nearestPoint() this is continuous in (x, y) rather than
  // snapped to a waypoint, so the horizon does not start up to one waypoint
  // spacing (2 m on traj_race_cl.csv) behind the vehicle.
  double projectedArcLength(double x, double y) const;

private:
  ReferencePoint interpolateAtArcLength(double s) const;

  // Backward (decel) then forward (accel) passes over the loaded profile, so
  // every corner speed is one the car can actually brake down to and build
  // back up from.
  void applyLongitudinalLimits();

  std::vector<ReferencePoint> waypoints_;
  double track_length_{0.0};
  double speed_limit_{std::numeric_limits<double>::infinity()};
  double lateral_accel_limit_{std::numeric_limits<double>::infinity()};
  double decel_limit_{0.0};
  double accel_limit_{0.0};
  size_t clamped_count_{0};
  size_t curvature_clamped_count_{0};
  double max_raw_speed_{0.0};
  double max_raw_lateral_demand_{0.0};
  double max_lateral_demand_{0.0};
  mutable size_t last_nearest_index_{0};
  mutable bool has_prior_nearest_{false};
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_
