#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "mpc_path_tracking/grip_limits.hpp"
#include "mpc_path_tracking/reference_trajectory_handler.hpp"

using mpc_path_tracking::ReferenceTrajectoryHandler;
using mpc_path_tracking::TireParams;
using mpc_path_tracking::VehicleParams;
namespace grip = mpc_path_tracking::grip;

namespace
{
VehicleParams testVehicle()
{
  VehicleParams vp;
  vp.mass = 269.84;
  vp.Iz = 51.1;
  vp.l_f = 0.738142;
  vp.l_r = 0.795362;
  return vp;
}

TireParams testTire()
{
  TireParams t;
  t.Bf = 10.0; t.Cf = 1.9; t.Df = 1.5; t.Ef = 0.0;
  t.Br = 10.0; t.Cr = 1.9; t.Dr = 1.5; t.Er = 0.0;
  return t;
}

// Straight-line geometry carrying a gaussian curvature bump, so the profile has
// to brake INTO rising curvature - the overlap the friction ellipse governs.
f1tenth_msgs::msg::WaypointArray makeCorner(double kappa_max, int count, double ds, double vx)
{
  f1tenth_msgs::msg::WaypointArray msg;
  msg.waypoints.reserve(count);
  const double centre = count / 2.0;
  const double width = count / 8.0;
  for (int i = 0; i < count; ++i) {
    const double z = (static_cast<double>(i) - centre) / width;
    f1tenth_msgs::msg::Waypoint wp;
    wp.s_m = i * ds;
    wp.x_m = i * ds;
    wp.y_m = 0.0;
    wp.psi_rad = 0.0;
    wp.kappa_radpm = kappa_max * std::exp(-z * z);
    wp.vx_mps = vx;
    wp.ax_mps2 = 0.0;
    msg.waypoints.push_back(wp);
  }
  return msg;
}

// Largest sqrt(ax^2 + ay^2) the loaded profile demands, as a fraction of `budget`.
// ax is differentiated from the profile itself, so this measures what the
// reference asks of the tires rather than what the handler intended.
double peakCombinedDemandRatio(const ReferenceTrajectoryHandler & ref, double ds, double budget)
{
  double worst = 0.0;
  const size_t n = ref.waypointCount();
  for (size_t i = 0; i + 1 < n; ++i) {
    const auto & p = ref.nearestPoint(i * ds, 0.0);
    const auto & q = ref.nearestPoint((i + 1) * ds, 0.0);
    const double ax = (q.vx * q.vx - p.vx * p.vx) / (2.0 * ds);
    const double ay = p.vx * p.vx * std::abs(p.kappa);
    worst = std::max(worst, std::hypot(ax, ay) / budget);
  }
  return worst;
}
}  // namespace

TEST(GripLimits, GripCeilingIsLoadWeightedPeakFriction)
{
  const VehicleParams vp = testVehicle();
  TireParams t = testTire();
  t.Df = 1.0;
  t.Dr = 1.0;
  // Both axles at D = 1 carry exactly one g however the load is split.
  EXPECT_NEAR(grip::gripCeiling(t, vp), 9.81, 1e-9);
}

TEST(GripLimits, PeakSlipAngleMatchesTheClosedFormAtZeroE)
{
  const double B = 10.0, C = 1.9;
  const double expected = std::tan(M_PI / (2.0 * C)) / B;
  EXPECT_NEAR(grip::peakSlipAngle(B, C, 1.5, 0.0), expected, 1e-3);
}

TEST(GripLimits, PeakSlipAngleSurvivesAnEAboveOne)
{
  // E > 1 makes the implicit peak equation non-monotone; the scan still lands on
  // a real maximum of the force curve.
  const double alpha = grip::peakSlipAngle(2.41, 4.81, 0.59, 5.0);
  const auto force = [](double a) {
      const double Ba = 2.41 * a;
      return 0.59 * std::sin(4.81 * std::atan(Ba - 5.0 * (Ba - std::atan(Ba))));
    };
  EXPECT_GT(alpha, 0.0);
  EXPECT_GE(force(alpha), force(alpha * 0.5));
  EXPECT_GE(force(alpha), force(std::min(alpha * 1.5, 0.5236)));
}

TEST(GripLimits, ShapeUtilizationIsUnchangedForTheReferenceTire)
{
  const TireParams t = testTire();
  EXPECT_DOUBLE_EQ(grip::shapeUtilization(t, t, 0.5), 0.5);
}

TEST(GripLimits, ShapeUtilizationFallsWhenThePeakMovesOut)
{
  const TireParams ref = testTire();
  TireParams soft = ref;
  soft.Bf = 5.0;  // half the stiffness, so the peak sits at twice the slip angle
  soft.Br = 5.0;
  EXPECT_LT(grip::shapeUtilization(soft, ref, 0.5), 0.5);
}

TEST(GripLimits, ShapeUtilizationNeverExceedsTheBase)
{
  const TireParams ref = testTire();
  TireParams stiff = ref;
  stiff.Bf = 40.0;
  stiff.Br = 40.0;
  EXPECT_DOUBLE_EQ(grip::shapeUtilization(stiff, ref, 0.5), 0.5);
}

TEST(GripLimits, SigmaTighteningShrinksWithUncertainty)
{
  EXPECT_DOUBLE_EQ(grip::applySigmaTightening(0.5, 1.0, 0.0, 1.0, 0.2), 0.5);
  EXPECT_NEAR(grip::applySigmaTightening(0.5, 1.0, 0.1, 1.0, 0.2), 0.45, 1e-12);
  EXPECT_LT(
    grip::applySigmaTightening(0.5, 1.0, 0.2, 1.0, 0.2),
    grip::applySigmaTightening(0.5, 1.0, 0.1, 1.0, 0.2));
  // Clamped at util_min however bad the fit gets.
  EXPECT_DOUBLE_EQ(grip::applySigmaTightening(0.5, 1.0, 5.0, 1.0, 0.2), 0.2);
}

TEST(GripLimits, SigmaTighteningIgnoresAnUnusableEstimate)
{
  EXPECT_DOUBLE_EQ(grip::applySigmaTightening(0.5, 0.0, 0.1, 1.0, 0.2), 0.5);
  EXPECT_DOUBLE_EQ(
    grip::applySigmaTightening(0.5, 1.0, std::nan(""), 1.0, 0.2), 0.5);
}

TEST(GripLimits, CeilingDropsAtOnceAndRecoversOnARamp)
{
  EXPECT_DOUBLE_EQ(grip::rateLimitedCeiling(0.0, 8.0, 0.5, 1.0), 8.0);   // first value
  EXPECT_DOUBLE_EQ(grip::rateLimitedCeiling(8.0, 4.0, 0.5, 1.0), 4.0);   // drop
  EXPECT_DOUBLE_EQ(grip::rateLimitedCeiling(4.0, 8.0, 0.5, 1.0), 4.5);   // ramped rise
  EXPECT_DOUBLE_EQ(grip::rateLimitedCeiling(4.0, 4.2, 0.5, 1.0), 4.2);   // rise inside the ramp
}

// With no fast estimate the budget is the identified D times the base
// utilization and nothing else - the configuration every arm without a friction
// warm start runs in.
TEST(GripLimits, BudgetWithNoFastEstimateIsTheIdentifiedDAlone)
{
  const VehicleParams vp = testVehicle();
  const TireParams t = testTire();
  const double base_util = 0.5;
  const double util = grip::shapeUtilization(t, t, base_util);
  EXPECT_DOUBLE_EQ(
    grip::gripCeiling(t, vp) * util, grip::gripCeiling(t, vp) * base_util);
}

TEST(ReferenceTrajectoryEllipse, KeepsCombinedDemandInsideTheBudget)
{
  constexpr double kDs = 1.0;
  constexpr double kBudget = 8.0;
  const auto corner = makeCorner(0.05, 200, kDs, 30.0);

  ReferenceTrajectoryHandler without;
  without.setSpeedLimit(30.0);
  without.setLateralAccelLimit(kBudget);
  without.setLongitudinalLimits(kBudget, kBudget);
  without.setFrictionEllipse(false);
  without.setWaypoints(corner);

  ReferenceTrajectoryHandler with;
  with.setSpeedLimit(30.0);
  with.setLateralAccelLimit(kBudget);
  with.setLongitudinalLimits(kBudget, kBudget);
  with.setFrictionEllipse(true);
  with.setWaypoints(corner);

  const double ratio_without = peakCombinedDemandRatio(without, kDs, kBudget);
  const double ratio_with = peakCombinedDemandRatio(with, kDs, kBudget);

  EXPECT_GT(ratio_without, 1.05) << "the uncoupled profile should overrun the grip budget";
  EXPECT_LE(ratio_with, 1.05) << "the ellipse should hold combined demand at the budget";
  EXPECT_LT(ratio_with, ratio_without);
}

TEST(AxleGripCeiling, MatchesTheSumOnlyForABalancedTireWithoutLoadTransfer)
{
  VehicleParams vp = testVehicle();
  vp.h_cg = 0.31538;
  TireParams t = testTire();

  EXPECT_NEAR(grip::axleGripCeiling(t, vp, 0.0), grip::gripCeiling(t, vp), 1e-9);

  // The weaker axle sets the limit; the stronger one's surplus is unreachable
  // because steady-state moment balance fixes both axles at F_y/F_z = a_y/g.
  t.Dr = 1.2;
  EXPECT_NEAR(grip::axleGripCeiling(t, vp, 0.0), t.Dr * grip::kGravity, 1e-9);
  EXPECT_LT(grip::axleGripCeiling(t, vp, 0.0), grip::gripCeiling(t, vp));
}

TEST(AxleGripCeiling, BrakingUnloadsTheRearAndShrinksTheLateralBudget)
{
  VehicleParams vp = testVehicle();
  vp.h_cg = 0.31538;
  const TireParams t = testTire();

  const double level = grip::axleGripCeiling(t, vp, 0.0);
  const double braking = grip::axleGripCeiling(t, vp, -3.71);
  EXPECT_LT(braking, level);
  // m*a_x*h_cg off the rear, against a demand the moment balance leaves untouched.
  EXPECT_NEAR(braking, t.Dr * (grip::kGravity * vp.l_f - 3.71 * vp.h_cg) / vp.l_f, 1e-9);

  // Accelerating loads the rear instead, and the front becomes the binding axle.
  EXPECT_LT(grip::axleGripCeiling(t, vp, 5.71), level);
}
