#ifndef TRACK_GEOMETRY_UTILS__TRACK_GEOMETRY_UTILS_HPP_
#define TRACK_GEOMETRY_UTILS__TRACK_GEOMETRY_UTILS_HPP_

#include <cstddef>

#include "f1tenth_msgs/msg/waypoint_array.hpp"

namespace track_geometry_utils
{

struct TrackError
{
  double e_y{0.0};
  double heading_error{0.0};
  size_t nearest_idx{0};
};

// Brute-force nearest-point projection of (x, y, yaw) onto `waypoints`
// (nearest by squared Euclidean distance over x_m/y_m). Stateless - no
// hint/wrap-around index kept between calls, unlike
// mpc_path_tracking::ReferenceTrajectoryHandler::nearestPoint(), which
// keeps a search hint for its hot horizon-building path. This function is
// for the once-per-tick error/health computation shared by
// mpc_path_tracking's debug output and adaptive_controller_manager's
// safety gates - never called on MPC's QP hot path.
//
// e_y: signed lateral error, positive to the left of the reference
// heading (e_y = -sin(psi)*dx + cos(psi)*dy).
// heading_error: yaw - psi at the nearest point, wrapped to (-pi, pi].
//
// Precondition: waypoints.waypoints must be non-empty.
TrackError computeTrackError(
  const f1tenth_msgs::msg::WaypointArray & waypoints,
  double x, double y, double yaw);

// Largest |kappa_radpm| among waypoints within `lookahead_distance` (m,
// accumulated as Euclidean distance between consecutive waypoints) ahead of
// `start_idx`, wrapping around the array as a closed loop. Used to detect an
// imminent sharp corner before starting a controller handover.
//
// Precondition: waypoints.waypoints must be non-empty; start_idx < size().
double maxCurvatureAhead(
  const f1tenth_msgs::msg::WaypointArray & waypoints,
  size_t start_idx, double lookahead_distance);

}  // namespace track_geometry_utils

#endif  // TRACK_GEOMETRY_UTILS__TRACK_GEOMETRY_UTILS_HPP_
