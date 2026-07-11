#include "track_geometry_utils/track_geometry_utils.hpp"

#include <cmath>
#include <limits>

namespace track_geometry_utils
{

TrackError computeTrackError(
  const f1tenth_msgs::msg::WaypointArray & waypoints,
  double x, double y, double yaw)
{
  TrackError result;
  if (waypoints.waypoints.empty()) {
    return result;
  }

  size_t nearest_idx = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
    const auto & wp = waypoints.waypoints[i];
    const double dx = x - wp.x_m;
    const double dy = y - wp.y_m;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      nearest_idx = i;
    }
  }

  const auto & nearest = waypoints.waypoints[nearest_idx];
  const double dx = x - nearest.x_m;
  const double dy = y - nearest.y_m;
  const double psi = nearest.psi_rad;

  result.nearest_idx = nearest_idx;
  result.e_y = -std::sin(psi) * dx + std::cos(psi) * dy;

  const double raw_diff = yaw - psi;
  result.heading_error = std::atan2(std::sin(raw_diff), std::cos(raw_diff));

  return result;
}

}  // namespace track_geometry_utils
