#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "mpc_path_tracking/vehicle_model.hpp"

using mpc_path_tracking::Input;
using mpc_path_tracking::State;
using mpc_path_tracking::TireParams;
using mpc_path_tracking::VehicleModel;
using mpc_path_tracking::VehicleParams;

namespace
{
VehicleModel makeSimModel()
{
  // Defaults from On-Track-SysID/models/SIM/SIM_pacejka.txt
  VehicleParams vp;
  vp.mass = 3.54;
  vp.Iz = 0.05797;
  vp.l_f = 0.162;
  vp.l_r = 0.145;
  vp.h_cg = 0.02;

  TireParams tp;
  tp.Bf = 2.4128;
  tp.Cf = 4.8155;
  tp.Df = 0.5922;
  tp.Ef = 5.0;
  tp.Br = 14.4445;
  tp.Cr = 1.2129;
  tp.Dr = 0.6842;
  tp.Er = 0.8526;

  return VehicleModel(vp, tp);
}

double referencePacejka(double alpha, double B, double C, double D, double E)
{
  const double Ba = B * alpha;
  return D * std::sin(C * std::atan(Ba - E * (Ba - std::atan(Ba))));
}
}  // namespace

TEST(VehicleModel, PacejkaZeroAtZeroSlip)
{
  const VehicleModel model = makeSimModel();
  double fz_f, fz_r;
  model.normalLoads(fz_f, fz_r);
  EXPECT_NEAR(model.lateralForceFront(0.0, fz_f), 0.0, 1e-9);
  EXPECT_NEAR(model.lateralForceRear(0.0, fz_r), 0.0, 1e-9);
}

TEST(VehicleModel, PacejkaMatchesHandComputedFormula)
{
  const VehicleModel model = makeSimModel();
  double fz_f, fz_r;
  model.normalLoads(fz_f, fz_r);

  const double alpha_f = 0.08;
  const double expected_f = fz_f * referencePacejka(alpha_f, 2.4128, 4.8155, 0.5922, 5.0);
  EXPECT_NEAR(model.lateralForceFront(alpha_f, fz_f), expected_f, 1e-9);

  const double alpha_r = -0.05;
  const double expected_r = fz_r * referencePacejka(alpha_r, 14.4445, 1.2129, 0.6842, 0.8526);
  EXPECT_NEAR(model.lateralForceRear(alpha_r, fz_r), expected_r, 1e-9);
}

TEST(VehicleModel, PacejkaOddSymmetry)
{
  const VehicleModel model = makeSimModel();
  double fz_f, fz_r;
  model.normalLoads(fz_f, fz_r);
  const double alpha = 0.1;
  EXPECT_NEAR(
    model.lateralForceFront(alpha, fz_f), -model.lateralForceFront(-alpha, fz_f), 1e-9);
}

TEST(VehicleModel, NormalLoadsSumToWeight)
{
  const VehicleModel model = makeSimModel();
  double fz_f, fz_r;
  model.normalLoads(fz_f, fz_r);
  EXPECT_NEAR(fz_f + fz_r, model.vehicleParams().mass * 9.81, 1e-6);
  EXPECT_GT(fz_f, 0.0);
  EXPECT_GT(fz_r, 0.0);
}

TEST(VehicleModel, SlipAnglesFiniteNearZeroSpeed)
{
  const VehicleModel model = makeSimModel();
  State x = State::Zero();
  x(3) = 0.0;   // vx = 0, would be singular without the internal floor clamp
  x(4) = 0.2;
  x(5) = 0.1;
  double alpha_f, alpha_r;
  model.slipAngles(x, 0.0, alpha_f, alpha_r);
  EXPECT_TRUE(std::isfinite(alpha_f));
  EXPECT_TRUE(std::isfinite(alpha_r));
}

TEST(VehicleModel, SlipAnglesMatchSysIdConvention)
{
  // Pins slipAngles() to the exact sign convention used by
  // On-Track-SysID/src/helpers/data_processing.py (compute_slip_angles),
  // which is what the identified Bf/Cf/Df/Ef/Br/Cr/Dr/Er coefficients were
  // fit against: alpha_f = delta - atan2(vy+lf*r, vx),
  // alpha_r = -atan2(vy-lr*r, vx). Getting this backwards flips the sign of
  // the tire force (Fy = D*sin(...) is odd in alpha) and silently turns a
  // stabilizing restoring force into a destabilizing one.
  const VehicleModel model = makeSimModel();
  const double lf = model.vehicleParams().l_f;
  const double lr = model.vehicleParams().l_r;

  State x;
  x << 0.0, 0.0, 0.0, 3.0, 0.3, 0.15;
  const double delta = 0.05;

  double alpha_f, alpha_r;
  model.slipAngles(x, delta, alpha_f, alpha_r);

  const double expected_f = delta - std::atan2(x(4) + lf * x(5), x(3));
  const double expected_r = -std::atan2(x(4) - lr * x(5), x(3));
  EXPECT_NEAR(alpha_f, expected_f, 1e-9);
  EXPECT_NEAR(alpha_r, expected_r, 1e-9);
}

TEST(VehicleModel, StraightLineConstantVelocityIntegration)
{
  const VehicleModel model = makeSimModel();
  State x = State::Zero();
  x(3) = 2.0;   // vx
  const Input u(0.0, 0.0);
  const double dt = 0.05;
  const State x_next = model.integrateRk4(x, u, dt);
  // Zero steering & zero slip -> zero lateral force -> vy, r stay ~0,
  // so X should advance by ~vx*dt.
  EXPECT_NEAR(x_next(0), x(3) * dt, 1e-6);
  EXPECT_NEAR(x_next(1), 0.0, 1e-6);
  EXPECT_NEAR(x_next(4), 0.0, 1e-6);
  EXPECT_NEAR(x_next(5), 0.0, 1e-6);
}

TEST(VehicleModel, LinearizeDiscreteReproducesNominalPoint)
{
  const VehicleModel model = makeSimModel();
  State x;
  x << 1.0, 2.0, 0.3, 3.0, 0.1, 0.2;
  const Input u(0.05, 0.5);
  const double dt = 0.05;

  mpc_path_tracking::StateJacobian Ad;
  mpc_path_tracking::InputJacobian Bd;
  State c;
  model.linearizeDiscrete(x, u, dt, Ad, Bd, c);

  // By construction, Ad*x + Bd*u + c must reproduce integrateRk4(x, u, dt)
  // exactly at the linearization point itself.
  const State affine_prediction = Ad * x + Bd * u + c;
  const State exact_prediction = model.integrateRk4(x, u, dt);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(affine_prediction(i), exact_prediction(i), 1e-6);
  }
}

TEST(VehicleModel, IntegrationSubstepsScaleWithStiffnessAndInverseSpeed)
{
  VehicleParams vp;
  vp.mass = 240.0;
  vp.Iz = 51.1;
  vp.l_f = 1.0653;
  vp.l_r = 1.0653;

  TireParams stiff;
  stiff.Bf = 20.0; stiff.Cf = 1.2; stiff.Df = 1.3107; stiff.Ef = -3.0;
  stiff.Br = 7.9577; stiff.Cr = 1.2; stiff.Dr = 1.04; stiff.Er = 0.5506;
  const VehicleModel stiff_model(vp, stiff);

  State slow, fast;
  slow << 0.0, 0.0, 0.0, 8.0, 0.0, 0.0;
  fast << 0.0, 0.0, 0.0, 40.0, 0.0, 0.0;
  EXPECT_GT(stiff_model.integrationSubsteps(slow, 0.05), 1);
  EXPECT_GT(
    stiff_model.integrationSubsteps(slow, 0.05), stiff_model.integrationSubsteps(fast, 0.05));
  EXPECT_LE(stiff_model.integrationSubsteps(slow, 0.05), VehicleModel::kMaxSubsteps);

  // A soft tire set at the same speed needs no sub-stepping, so the cost is
  // only paid where the dynamics actually are stiff.
  TireParams soft = stiff;
  soft.Bf = 2.0; soft.Df = 0.6; soft.Br = 2.0; soft.Dr = 0.6;
  EXPECT_LT(
    VehicleModel(vp, soft).integrationSubsteps(slow, 0.05),
    stiff_model.integrationSubsteps(slow, 0.05));
}

// Pins the mechanism behind the "QP solve failed" fallbacks seen right after
// a PP->MPC switch: the sysid run that produced SIM_pacejka.txt fitted a
// front axle roughly 3x stiffer than the rear on a 50/50 CG, i.e. a strongly
// oversteering car. The linear single-track model is then open-loop unstable
// above v_crit = L*sqrt(Cf*Cr/(m*(l_f*Cf - l_r*Cr))) ~= 17 m/s, and since
// MpcController linearizes each stage about the *reference* speed
// (14.9-46 m/s on traj_race_cl.csv) most of the horizon sits above it.
// rho(Ad) > 1 compounds over N stages until the condensed QP is numerically
// meaningless. Understeering params keep every stage contractive.
TEST(VehicleModel, OversteeringTireSetMakesLinearizationUnstableAboveCriticalSpeed)
{
  VehicleParams vp;      // CARLA asurt_fsai, as configured in mpc_path_tracking.yaml
  vp.mass = 240.0;
  vp.Iz = 51.1;
  vp.l_f = 1.0653;
  vp.l_r = 1.0653;

  TireParams identified;   // On-Track-SysID output pushed via mpc/update_params
  identified.Bf = 20.0; identified.Cf = 1.2; identified.Df = 1.3107; identified.Ef = -3.0;
  identified.Br = 7.9577; identified.Cr = 1.2; identified.Dr = 1.04; identified.Er = 0.5506;

  const VehicleModel model(vp, identified);
  double fz_f, fz_r;
  model.normalLoads(fz_f, fz_r);
  const double c_front = fz_f * identified.Bf * identified.Cf * identified.Df;
  const double c_rear = fz_r * identified.Br * identified.Cr * identified.Dr;
  ASSERT_GT(vp.l_f * c_front - vp.l_r * c_rear, 0.0) << "expected an oversteering fit";
  const double v_crit = (vp.l_f + vp.l_r) *
    std::sqrt(c_front * c_rear / (vp.mass * (vp.l_f * c_front - vp.l_r * c_rear)));
  EXPECT_NEAR(v_crit, 17.4, 0.5);

  auto spectral_radius = [&model](double vx) {
      State x;
      x << 0.0, 0.0, 0.0, vx, 0.0, 0.0;
      const Input u(0.0, 0.0);
      mpc_path_tracking::StateJacobian Ad;
      mpc_path_tracking::InputJacobian Bd;
      State c;
      model.linearizeDiscrete(x, u, 0.05, Ad, Bd, c);
      return Eigen::EigenSolver<mpc_path_tracking::StateJacobian>(Ad, false)
             .eigenvalues().cwiseAbs().maxCoeff();
    };

  // Below v_crit the lateral modes are contractive (the two position states
  // contribute the marginal unit eigenvalues, hence the tolerance).
  EXPECT_LE(spectral_radius(10.0), 1.0 + 1e-6);
  // Above it the reference-speed stages of the horizon diverge outright.
  EXPECT_GT(spectral_radius(30.0), 1.3);
  EXPECT_GT(spectral_radius(46.0), 1.7);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace
{
// The CARLA vehicle.vehicle.asurt_fsai the stack currently drives, with the
// yaml startup tire prior. This is the configuration in which the high-speed
// steering limit cycle was observed.
VehicleModel makeCarlaModel()
{
  VehicleParams vp;
  vp.mass = 240.0;
  vp.Iz = 51.1;
  vp.l_f = 0.738142;
  vp.l_r = 0.795362;
  vp.h_cg = 0.31538;

  TireParams tp;
  tp.Bf = 10.0; tp.Cf = 1.9; tp.Df = 1.5; tp.Ef = 0.97;
  tp.Br = 10.0; tp.Cr = 1.9; tp.Dr = 1.5; tp.Er = 0.97;
  return VehicleModel(vp, tp);
}
}  // namespace

// Regression: bug-mpc-highspeed-infeasible-reference. steadyStateCornering must
// return a point at which the lateral dynamics are genuinely in equilibrium -
// that is the whole reason it exists. Previously MpcController linearized about
// (vy = 0, r = v*kappa, delta = atan(L*kappa)), where vy_dot was -7.40 m/s^2 at
// 27 m/s, i.e. not an equilibrium at all.
TEST(VehicleModel, SteadyStateCorneringIsAnActualEquilibrium)
{
  const VehicleModel model = makeCarlaModel();

  // Sweep the (v, kappa) envelope the raceline actually demands: traj_race_cl.csv
  // peaks at a_y = 12.0 m/s^2 whatever the speed cap (against ~14.7 available)
  // and at kappa = 0.0536 1/m. Outside that envelope there is no steady state
  // to find - see the Unachievable test.
  constexpr double kRacelineKappaMax = 0.0536;
  for (double v : {5.0, 11.11, 20.0, 27.0, 30.0}) {
    for (double a_y : {0.0, 1.0, 6.0, 12.0}) {
      const double kappa = std::min(a_y / (v * v), kRacelineKappaMax);
      const auto ss = model.steadyStateCornering(v, kappa);
      ASSERT_TRUE(ss.converged) << "v=" << v << " kappa=" << kappa;

      State x;
      x << 0.0, 0.0, 0.0, v, ss.vy, v * kappa;
      const Input u(ss.delta, 0.0);
      const State xdot = model.continuousDynamics(x, u);

      EXPECT_NEAR(xdot(4), 0.0, 1e-6) << "vy_dot at v=" << v << " kappa=" << kappa;
      EXPECT_NEAR(xdot(5), 0.0, 1e-6) << "r_dot at v=" << v << " kappa=" << kappa;
    }
  }
}

// Sideslip is what makes the old zero-vy reference wrong, and it is negligible
// at low speed and large at high speed - which is exactly why the oscillation
// only showed up once limits.speed_max was raised.
TEST(VehicleModel, SteadyStateSideslipGrowsWithSpeed)
{
  const VehicleModel model = makeCarlaModel();
  const double kappa = 0.0145;

  const auto slow = model.steadyStateCornering(11.11, kappa);
  const auto fast = model.steadyStateCornering(27.0, kappa);

  ASSERT_TRUE(slow.converged);
  ASSERT_TRUE(fast.converged);

  EXPECT_LT(std::abs(slow.vy), 0.15) << "at 11 m/s the old vy_ref = 0 was harmless";
  EXPECT_GT(std::abs(fast.vy), 0.5) << "at 27 m/s it is not";
  EXPECT_GT(std::abs(fast.beta), 3.0 * std::abs(slow.beta));
}

// Straight-line driving must return exactly zero, so the reference is unchanged
// on the parts of the track where the old formulation was already correct.
TEST(VehicleModel, SteadyStateCorneringZeroOnStraightLine)
{
  const VehicleModel model = makeCarlaModel();
  const auto ss = model.steadyStateCornering(27.0, 0.0);

  EXPECT_TRUE(ss.converged);
  EXPECT_DOUBLE_EQ(ss.vy, 0.0);
  EXPECT_DOUBLE_EQ(ss.delta, 0.0);
  EXPECT_DOUBLE_EQ(ss.beta, 0.0);
}

// A turn beyond the tire model's grip has no steady state; it must be reported
// rather than returning a bogus "equilibrium" the MPC would then track.
TEST(VehicleModel, SteadyStateCorneringReportsUnachievableTurns)
{
  const VehicleModel model = makeCarlaModel();
  // 40 m/s on kappa = 0.05 needs 80 m/s^2 of lateral accel against ~14.7 available.
  const auto ss = model.steadyStateCornering(40.0, 0.05);
  EXPECT_FALSE(ss.converged);
}
