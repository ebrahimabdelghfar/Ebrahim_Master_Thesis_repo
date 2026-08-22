#include <cmath>
#include <memory>

#include <gtest/gtest.h>

#include "mpc_path_tracking/mpc_controller.hpp"
#include "mpc_path_tracking/reference_trajectory_handler.hpp"
#include "mpc_path_tracking/solver_interface.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

using mpc_path_tracking::AcadosMpcSolver;
using mpc_path_tracking::AcadosSolverSettings;
using mpc_path_tracking::Input;
using mpc_path_tracking::MpcConfig;
using mpc_path_tracking::MpcController;
using mpc_path_tracking::ReferenceTrajectoryHandler;
using mpc_path_tracking::State;
using mpc_path_tracking::TireParams;
using mpc_path_tracking::VehicleModel;
using mpc_path_tracking::VehicleParams;

namespace
{
VehicleParams carlaVehicle()
{
  VehicleParams v;
  v.mass = 240.0;
  v.Iz = 51.1;
  v.l_f = 0.738142;
  v.l_r = 0.795362;
  v.h_cg = 0.31538;
  return v;
}

TireParams carlaTire()
{
  TireParams t;
  t.Bf = 10.0; t.Cf = 1.9; t.Df = 1.5; t.Ef = 0.97;
  t.Br = 10.0; t.Cr = 1.9; t.Dr = 1.5; t.Er = 0.97;
  return t;
}

MpcConfig testConfig()
{
  MpcConfig c;
  c.N = 20;             // short - these tests are about the reference, not the horizon
  c.dt_min = 0.05;
  c.dt_max = 0.07;
  c.horizon_distance_m = 5.5;
  c.cost.Q << 20.0, 10.0, 20.0, 10.0, 10.0;
  c.cost.Qf << 20.0, 10.0, 20.0, 10.0, 10.0;
  c.cost.R << 100.0, 1.0;
  c.cost.Rrate << 100.0, 10.0;
  c.limits.steering_min = -0.2793;
  c.limits.steering_max = 0.2793;
  c.limits.steering_rate_max = 1.0;
  c.limits.accel_min = -14.71;
  c.limits.accel_max = 14.71;
  c.limits.jerk_max = 1000.0;
  return c;
}

std::unique_ptr<MpcController> makeController(const MpcConfig & cfg)
{
  AcadosSolverSettings acados;
  acados.qp_solver = "FULL_CONDENSING_QPOASES";
  acados.iter_max = 1000;
  return std::make_unique<MpcController>(
    VehicleModel(carlaVehicle(), carlaTire()),
    std::make_unique<AcadosMpcSolver>(cfg.N, acados), cfg);
}

// Counter-clockwise circle of radius R at constant speed: constant curvature,
// so the steady-state cornering solution is the same at every waypoint.
f1tenth_msgs::msg::WaypointArray makeCircle(double radius, double vx, int count)
{
  f1tenth_msgs::msg::WaypointArray msg;
  const double ds = 2.0 * M_PI * radius / count;
  for (int i = 0; i < count; ++i) {
    const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(count);
    f1tenth_msgs::msg::Waypoint w;
    w.s_m = i * ds;
    w.x_m = radius * std::cos(theta);
    w.y_m = radius * std::sin(theta);
    w.psi_rad = theta + M_PI_2;      // path tangent
    w.kappa_radpm = 1.0 / radius;
    w.vx_mps = vx;
    w.ax_mps2 = 0.0;
    msg.waypoints.push_back(w);
  }
  return msg;
}
}  // namespace

// Regression: bug-mpc-highspeed-infeasible-reference. A vehicle sitting exactly
// on the dynamically feasible reference is already doing everything the cost
// asks, so the controller must be content to hold the steering angle that
// sustains the turn. Under the old formulation (vy_ref = 0, psi_ref = the path
// tangent) this configuration carried a large cost - the controller was being
// told to fix a "problem" that was the car cornering correctly - and it is that
// contradiction that produced the steering chatter at speed.
TEST(MpcController, HoldsSteadyStateOnAConstantRadiusTurn)
{
  const double radius = 200.0;     // kappa = 0.005, well inside grip at 27 m/s
  const double vx = 27.0;

  const VehicleModel model(carlaVehicle(), carlaTire());
  const auto ss = model.steadyStateCornering(vx, 1.0 / radius);
  ASSERT_TRUE(ss.converged);

  ReferenceTrajectoryHandler ref;
  ref.setWaypoints(makeCircle(radius, vx, 400));

  const MpcConfig cfg = testConfig();
  auto ctrl = makeController(cfg);

  // Place the car exactly at the steady state at theta = 0: on the circle,
  // heading beta off the tangent, carrying the sideslip the turn requires.
  State x;
  x << radius, 0.0, M_PI_2 - ss.beta, vx, ss.vy, vx / radius;
  const Input u_prev(ss.delta, 0.0);

  const auto out = ctrl->computeCommand(x, u_prev, ref);
  ASSERT_TRUE(out.solved) << out.status;

  // The command should be the steering angle that sustains the turn, not a
  // correction away from it.
  EXPECT_NEAR(out.u0(0), ss.delta, 5e-3);
}

// The same car on the same turn must not need a materially different command
// at low speed - this pins that the fix did not break the regime that already
// worked, where beta ~ 0 and the old and new references nearly coincide.
TEST(MpcController, HoldsSteadyStateAtLowSpeedToo)
{
  const double radius = 60.0;
  const double vx = 11.11;

  const VehicleModel model(carlaVehicle(), carlaTire());
  const auto ss = model.steadyStateCornering(vx, 1.0 / radius);
  ASSERT_TRUE(ss.converged);
  EXPECT_LT(std::abs(ss.beta), 0.02) << "sideslip should be negligible here";

  ReferenceTrajectoryHandler ref;
  ref.setWaypoints(makeCircle(radius, vx, 400));

  auto ctrl = makeController(testConfig());
  State x;
  x << radius, 0.0, M_PI_2 - ss.beta, vx, ss.vy, vx / radius;

  const auto out = ctrl->computeCommand(x, Input(ss.delta, 0.0), ref);
  ASSERT_TRUE(out.solved) << out.status;
  EXPECT_NEAR(out.u0(0), ss.delta, 5e-3);
}

// Pins that every stage is linearized about the reference EQUILIBRIUM, never
// about the previous solution. Successive linearization is the textbook choice
// and was implemented and measured here: at 27 m/s with one tick of transport
// delay it took solve failures from 0/600 to 102/600 and the mean solve from
// ~15 ms to ~90 ms, because the previous solution's far stages sit nowhere
// near an equilibrium and this tire set puts rho(Ad) up to 2.1e6 there. If a
// future change reintroduces it, this test should fail and send the reader to
// the note in MpcController::computeCommand.
TEST(MpcController, AlwaysLinearizesAboutTheReferenceEquilibrium)
{
  const double radius = 200.0;
  const double vx = 27.0;

  ReferenceTrajectoryHandler ref;
  ref.setWaypoints(makeCircle(radius, vx, 400));

  auto ctrl = makeController(testConfig());
  State x;
  x << radius, 0.0, M_PI_2, vx, 0.0, vx / radius;

  const auto first = ctrl->computeCommand(x, Input::Zero(), ref);
  ASSERT_TRUE(first.solved) << first.status;
  EXPECT_FALSE(first.relinearized);

  const auto second = ctrl->computeCommand(x, first.u0, ref);
  ASSERT_TRUE(second.solved) << second.status;
  EXPECT_FALSE(second.relinearized) << "successive linearization must stay disabled";
}

// A turn the tire model cannot sustain must be counted and reported rather
// than silently tracked against a fabricated equilibrium.
TEST(MpcController, ReportsInfeasibleReferenceStages)
{
  const double radius = 25.0;      // 27 m/s on R=25 needs ~29 m/s^2, grip is ~14.7
  const double vx = 27.0;

  ReferenceTrajectoryHandler ref;
  ref.setWaypoints(makeCircle(radius, vx, 400));

  auto ctrl = makeController(testConfig());
  State x;
  x << radius, 0.0, M_PI_2, vx, 0.0, vx / radius;

  const auto out = ctrl->computeCommand(x, Input::Zero(), ref);
  EXPECT_GT(out.infeasible_ref_stages, 0);
}
