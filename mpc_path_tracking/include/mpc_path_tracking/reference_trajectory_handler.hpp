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

  // Number of waypoints whose vx_mps exceeded speed_limit_ and were clamped
  // by the last setWaypoints() call, and the largest speed seen. Non-zero
  // means the raceline does not match the vehicle it is being driven on.
  size_t clampedWaypointCount() const { return clamped_count_; }
  double maxRawSpeed() const { return max_raw_speed_; }

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
  // (x, y) onto the raceline, spaced by accumulated arc length
  // s_k+1 = s_k + vx_ref_k * dt, wrapping around the (closed) track length.
  // psi is unwrapped to stay continuous with `psi_hint` (the vehicle's
  // current heading) so downstream heading-error terms never see a 2*pi
  // jump.
  std::vector<ReferencePoint> buildHorizon(
    double x, double y, double psi_hint, int horizon_steps, double dt) const;

  // Arc length of the perpendicular projection of (x, y) onto the raceline
  // polyline. Unlike nearestPoint() this is continuous in (x, y) rather than
  // snapped to a waypoint, so the horizon does not start up to one waypoint
  // spacing (2 m on traj_race_cl.csv) behind the vehicle.
  double projectedArcLength(double x, double y) const;

private:
  ReferencePoint interpolateAtArcLength(double s) const;

  std::vector<ReferencePoint> waypoints_;
  double track_length_{0.0};
  double speed_limit_{std::numeric_limits<double>::infinity()};
  size_t clamped_count_{0};
  double max_raw_speed_{0.0};
  double max_lateral_demand_{0.0};
  mutable size_t last_nearest_index_{0};
  mutable bool has_prior_nearest_{false};
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_
