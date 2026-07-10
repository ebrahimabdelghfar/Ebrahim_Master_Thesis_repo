#ifndef MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_
#define MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_

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

private:
  ReferencePoint interpolateAtArcLength(double s) const;

  std::vector<ReferencePoint> waypoints_;
  double track_length_{0.0};
  mutable size_t last_nearest_index_{0};
  mutable bool has_prior_nearest_{false};
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__REFERENCE_TRAJECTORY_HANDLER_HPP_
