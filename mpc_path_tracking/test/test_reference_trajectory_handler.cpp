#include <cmath>

#include <gtest/gtest.h>

#include "mpc_path_tracking/reference_trajectory_handler.hpp"

using mpc_path_tracking::ReferenceTrajectoryHandler;

namespace
{
f1tenth_msgs::msg::WaypointArray makeStraightLine(double spacing, int count, double vx)
{
  f1tenth_msgs::msg::WaypointArray msg;
  msg.waypoints.reserve(count);
  for (int i = 0; i < count; ++i) {
    f1tenth_msgs::msg::Waypoint wp;
    wp.s_m = i * spacing;
    wp.x_m = i * spacing;
    wp.y_m = 0.0;
    wp.psi_rad = 0.0;
    wp.kappa_radpm = 0.0;
    wp.vx_mps = vx;
    wp.ax_mps2 = 0.0;
    msg.waypoints.push_back(wp);
  }
  return msg;
}

f1tenth_msgs::msg::WaypointArray makeCircle(double radius, int count, double vx)
{
  f1tenth_msgs::msg::WaypointArray msg;
  msg.waypoints.reserve(count);
  const double circumference = 2.0 * M_PI * radius;
  const double ds = circumference / count;
  for (int i = 0; i < count; ++i) {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    f1tenth_msgs::msg::Waypoint wp;
    wp.s_m = i * ds;
    wp.x_m = radius * std::cos(theta);
    wp.y_m = radius * std::sin(theta);
    wp.psi_rad = theta + M_PI / 2.0;   // tangent direction, counter-clockwise
    wp.kappa_radpm = 1.0 / radius;
    wp.vx_mps = vx;
    wp.ax_mps2 = 0.0;
    msg.waypoints.push_back(wp);
  }
  return msg;
}
}  // namespace

TEST(ReferenceTrajectoryHandler, EmptyBeforeWaypointsSet)
{
  ReferenceTrajectoryHandler handler;
  EXPECT_FALSE(handler.hasWaypoints());
}

TEST(ReferenceTrajectoryHandler, NearestPointOnStraightLine)
{
  ReferenceTrajectoryHandler handler;
  handler.setWaypoints(makeStraightLine(1.0, 20, 2.0));
  ASSERT_TRUE(handler.hasWaypoints());

  const auto nearest = handler.nearestPoint(5.3, 0.4);
  EXPECT_NEAR(nearest.x, 5.0, 1e-9);
  EXPECT_NEAR(nearest.y, 0.0, 1e-9);
}

TEST(ReferenceTrajectoryHandler, BuildHorizonAdvancesByReferenceSpeed)
{
  ReferenceTrajectoryHandler handler;
  const double vx = 2.0;
  handler.setWaypoints(makeStraightLine(0.1, 500, vx));
  ASSERT_TRUE(handler.hasWaypoints());

  const int N = 10;
  const double dt = 0.05;
  const auto horizon = handler.buildHorizon(0.0, 0.0, 0.0, N, dt);
  ASSERT_EQ(static_cast<int>(horizon.size()), N + 1);

  for (int k = 0; k <= N; ++k) {
    EXPECT_NEAR(horizon[k].x, vx * dt * k, 0.15);
    EXPECT_NEAR(horizon[k].y, 0.0, 1e-6);
    EXPECT_NEAR(horizon[k].vx, vx, 1e-9);
  }
}

TEST(ReferenceTrajectoryHandler, ConstantCurvatureArcKappaAndHeading)
{
  ReferenceTrajectoryHandler handler;
  const double radius = 5.0;
  handler.setWaypoints(makeCircle(radius, 720, 3.0));
  ASSERT_TRUE(handler.hasWaypoints());

  const auto horizon = handler.buildHorizon(radius, 0.0, M_PI / 2.0, 5, 0.02);
  ASSERT_EQ(horizon.size(), 6u);
  for (const auto & rp : horizon) {
    EXPECT_NEAR(rp.kappa, 1.0 / radius, 1e-6);
    // Heading should stay continuous (no wrap jump) and close to tangent direction.
    EXPECT_TRUE(std::isfinite(rp.psi));
  }
  // Heading should be monotonically increasing (no backwards 2*pi jump) since
  // the vehicle travels a small arc forward along the circle.
  for (size_t k = 1; k < horizon.size(); ++k) {
    EXPECT_GE(horizon[k].psi, horizon[k - 1].psi - 1e-6);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
