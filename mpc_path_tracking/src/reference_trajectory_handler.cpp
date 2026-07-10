#include "mpc_path_tracking/reference_trajectory_handler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mpc_path_tracking
{

namespace
{
double unwrapToNear(double angle, double reference)
{
  while (angle - reference > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle - reference < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}
}  // namespace

void ReferenceTrajectoryHandler::setWaypoints(const f1tenth_msgs::msg::WaypointArray & msg)
{
  waypoints_.clear();
  waypoints_.reserve(msg.waypoints.size());
  for (const auto & wp : msg.waypoints) {
    ReferencePoint rp;
    rp.s = wp.s_m;
    rp.x = wp.x_m;
    rp.y = wp.y_m;
    rp.psi = wp.psi_rad;
    rp.kappa = wp.kappa_radpm;
    rp.vx = wp.vx_mps;
    rp.ax = wp.ax_mps2;
    waypoints_.push_back(rp);
  }
  last_nearest_index_ = 0;
  has_prior_nearest_ = false;

  if (waypoints_.size() >= 2) {
    const auto & first = waypoints_.front();
    const auto & last = waypoints_.back();
    const double closing_gap = std::hypot(first.x - last.x, first.y - last.y);
    track_length_ = last.s + closing_gap;
  } else {
    track_length_ = 0.0;
  }
}

ReferencePoint ReferenceTrajectoryHandler::nearestPoint(double x, double y) const
{
  const size_t n = waypoints_.size();
  double best_dist_sq = std::numeric_limits<double>::infinity();
  size_t best_index = 0;

  // After the first fix, constrain the search to a window around the
  // previous match instead of a global scan. A global nearest-Euclidean-
  // point search can jump to a spatially-close-but-arc-length-far index
  // whenever the track passes near itself (hairpins, chicanes, the
  // start/finish straight next to a pit lane, etc.) -- since the whole
  // reference horizon is walked forward in arc length from this index,
  // one bad jump flips the reference heading/curvature and the MPC sees
  // the path pointing backward. A local, hysteresis-like window keeps the
  // match continuous with the vehicle's actual progress along the track.
  if (has_prior_nearest_ && n > 0) {
    constexpr size_t kSearchWindow = 60;
    const size_t half = std::min(kSearchWindow, n / 2 == 0 ? n : n / 2);
    for (size_t offset = 0; offset <= 2 * half; ++offset) {
      const size_t i = (last_nearest_index_ + n + offset - half) % n;
      const double dx = waypoints_[i].x - x;
      const double dy = waypoints_[i].y - y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = i;
      }
    }
  } else {
    for (size_t i = 0; i < n; ++i) {
      const double dx = waypoints_[i].x - x;
      const double dy = waypoints_[i].y - y;
      const double dist_sq = dx * dx + dy * dy;
      if (dist_sq < best_dist_sq) {
        best_dist_sq = dist_sq;
        best_index = i;
      }
    }
  }

  last_nearest_index_ = best_index;
  has_prior_nearest_ = true;
  return waypoints_[best_index];
}

ReferencePoint ReferenceTrajectoryHandler::interpolateAtArcLength(double s) const
{
  double s_wrapped = std::fmod(s, track_length_);
  if (s_wrapped < 0.0) {
    s_wrapped += track_length_;
  }

  // Binary search for the segment [i, i+1) with waypoints_[i].s <= s_wrapped.
  size_t lo = 0;
  size_t hi = waypoints_.size() - 1;
  while (lo + 1 < hi) {
    const size_t mid = lo + (hi - lo) / 2;
    if (waypoints_[mid].s <= s_wrapped) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  const ReferencePoint & p0 = waypoints_[lo];
  const bool wraps = (lo + 1 >= waypoints_.size());
  const ReferencePoint & p1 = wraps ? waypoints_.front() : waypoints_[lo + 1];

  const double s0 = p0.s;
  const double s1 = wraps ? track_length_ : p1.s;
  const double span = std::max(s1 - s0, 1e-9);
  const double alpha = std::clamp((s_wrapped - s0) / span, 0.0, 1.0);

  ReferencePoint out;
  out.s = s_wrapped;
  out.x = p0.x + alpha * (p1.x - p0.x);
  out.y = p0.y + alpha * (p1.y - p0.y);
  const double psi1_unwrapped = unwrapToNear(p1.psi, p0.psi);
  out.psi = p0.psi + alpha * (psi1_unwrapped - p0.psi);
  out.kappa = p0.kappa + alpha * (p1.kappa - p0.kappa);
  out.vx = p0.vx + alpha * (p1.vx - p0.vx);
  out.ax = p0.ax + alpha * (p1.ax - p0.ax);
  return out;
}

std::vector<ReferencePoint> ReferenceTrajectoryHandler::buildHorizon(
  double x, double y, double psi_hint, int horizon_steps, double dt) const
{
  std::vector<ReferencePoint> horizon;
  horizon.reserve(horizon_steps + 1);
  if (waypoints_.empty() || track_length_ <= 0.0) {
    return horizon;
  }

  const ReferencePoint start = nearestPoint(x, y);
  double s = start.s;
  double prev_psi = psi_hint;

  for (int k = 0; k <= horizon_steps; ++k) {
    ReferencePoint rp = interpolateAtArcLength(s);
    rp.psi = unwrapToNear(rp.psi, prev_psi);
    prev_psi = rp.psi;
    horizon.push_back(rp);
    s += std::max(rp.vx, 0.1) * dt;
  }
  return horizon;
}

}  // namespace mpc_path_tracking
