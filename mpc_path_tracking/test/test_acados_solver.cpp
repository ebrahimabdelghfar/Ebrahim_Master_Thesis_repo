#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "mpc_path_tracking/solver_interface.hpp"

using mpc_path_tracking::AcadosMpcSolver;
using mpc_path_tracking::AcadosSolverSettings;
using mpc_path_tracking::Input;
using mpc_path_tracking::MpcStage;
using mpc_path_tracking::OsqpMpcSolver;
using mpc_path_tracking::OsqpSolverSettings;
using mpc_path_tracking::SolverProblem;
using mpc_path_tracking::SolverSolution;
using mpc_path_tracking::State;
using mpc_path_tracking::StateJacobian;

namespace
{

// A small synthetic (not vehicle-derived) LTV-MPC problem, just structured
// enough to exercise the same code paths MpcController feeds into
// SolverInterface::solve(): non-identity dynamics, non-diagonal-only cost,
// box + rate limits. u_rate_max/u_min_box are the two knobs the three test
// cases below tighten to force each constraint type to actually bind, since
// a solver bug in the augmented-state formulation (see AcadosMpcSolver)
// would most likely surface as a mismatch precisely when a constraint is
// active, not in the unconstrained interior.
SolverProblem buildProblem(double u_rate_max, double u_box_max)
{
  constexpr int N = 5;
  const double dt = 0.05;

  SolverProblem problem;
  problem.N = N;
  problem.dt = dt;
  problem.x0 = State::Zero();
  problem.x0(0) = 1.0;   // e_y
  problem.x0(1) = 0.3;   // e_psi
  problem.x0(3) = 2.0;   // vx_err

  StateJacobian Ad = StateJacobian::Identity();
  Ad(0, 1) = 0.02;   // small state coupling, not just a pure diagonal decay
  Eigen::Matrix<double, 6, 2> Bd = Eigen::Matrix<double, 6, 2>::Zero();
  Bd(1, 0) = dt;
  Bd(3, 1) = dt;

  StateJacobian Qx = StateJacobian::Zero();
  Qx.diagonal() << 200.0, 200.0, 50.0, 10.0, 10.0, 5.0;

  problem.stages.resize(N);
  for (int k = 0; k < N; ++k) {
    MpcStage & s = problem.stages[k];
    s.Ad = Ad;
    s.Bd = Bd;
    s.c = State::Zero();
    s.Qx = Qx;
    s.qx = State::Zero();
    s.cost_offset = 0.0;
  }
  problem.Qx_terminal = Qx;
  problem.qx_terminal = State::Zero();
  problem.cost_offset = 0.0;

  problem.R = Eigen::Matrix2d::Identity() * 5.0;
  problem.Rrate = Eigen::Matrix2d::Identity() * 2.0;
  problem.u_prev = Input::Zero();
  problem.u_min = Input(-u_box_max, -u_box_max);
  problem.u_max = Input(u_box_max, u_box_max);
  problem.u_rate_min = Input(-u_rate_max, -u_rate_max);
  problem.u_rate_max = Input(u_rate_max, u_rate_max);
  return problem;
}

void expectSolutionsAgree(const SolverSolution & a, const SolverSolution & b)
{
  ASSERT_TRUE(a.solved);
  ASSERT_TRUE(b.solved);
  ASSERT_EQ(a.u.size(), b.u.size());
  ASSERT_EQ(a.x.size(), b.x.size());

  for (size_t k = 0; k < a.u.size(); ++k) {
    EXPECT_NEAR(a.u[k](0), b.u[k](0), 2e-3) << "u[" << k << "](0)";
    EXPECT_NEAR(a.u[k](1), b.u[k](1), 2e-3) << "u[" << k << "](1)";
  }
  for (size_t k = 0; k < a.x.size(); ++k) {
    for (int i = 0; i < 6; ++i) {
      EXPECT_NEAR(a.x[k](i), b.x[k](i), 5e-3) << "x[" << k << "](" << i << ")";
    }
  }
  EXPECT_NEAR(a.cost, b.cost, 1e-2 * std::max(1.0, std::abs(a.cost)));
}

std::pair<SolverSolution, SolverSolution> solveBoth(const SolverProblem & problem)
{
  OsqpSolverSettings osqp_settings;
  OsqpMpcSolver osqp(osqp_settings);

  AcadosSolverSettings acados_settings;
  AcadosMpcSolver acados(problem.N, acados_settings);

  SolverSolution osqp_solution;
  SolverSolution acados_solution;
  osqp.solve(problem, osqp_solution);
  acados.solve(problem, acados_solution);
  return {osqp_solution, acados_solution};
}

}  // namespace

TEST(AcadosMpcSolver, AgreesWithOsqpUnconstrained)
{
  const auto problem = buildProblem(/*u_rate_max=*/10.0, /*u_box_max=*/10.0);
  const auto [osqp_solution, acados_solution] = solveBoth(problem);
  expectSolutionsAgree(osqp_solution, acados_solution);
}

TEST(AcadosMpcSolver, AgreesWithOsqpRateLimitBinding)
{
  const auto problem = buildProblem(/*u_rate_max=*/0.15, /*u_box_max=*/10.0);
  const auto [osqp_solution, acados_solution] = solveBoth(problem);
  expectSolutionsAgree(osqp_solution, acados_solution);
}

TEST(AcadosMpcSolver, AgreesWithOsqpBoxLimitBinding)
{
  const auto problem = buildProblem(/*u_rate_max=*/10.0, /*u_box_max=*/0.2);
  const auto [osqp_solution, acados_solution] = solveBoth(problem);
  expectSolutionsAgree(osqp_solution, acados_solution);
}

TEST(AcadosMpcSolver, ReportsStatusOnSuccess)
{
  const auto problem = buildProblem(/*u_rate_max=*/10.0, /*u_box_max=*/10.0);
  AcadosSolverSettings settings;
  AcadosMpcSolver acados(problem.N, settings);
  SolverSolution solution;
  ASSERT_TRUE(acados.solve(problem, solution));
  EXPECT_EQ(solution.status, "OK");
  EXPECT_EQ(solution.status_code, 0);
}

// A diverging linearization (or any NaN leaking out of the tire model) must
// be caught before it reaches acados, and must say which stage was bad -
// otherwise every failure mode reads as the same "QP solve failed".
TEST(AcadosMpcSolver, RejectsNonFiniteStageDataWithDiagnosis)
{
  auto problem = buildProblem(/*u_rate_max=*/10.0, /*u_box_max=*/10.0);
  problem.stages[3].Ad(2, 2) = std::numeric_limits<double>::quiet_NaN();

  AcadosSolverSettings settings;
  AcadosMpcSolver acados(problem.N, settings);
  SolverSolution solution;
  EXPECT_FALSE(acados.solve(problem, solution));
  EXPECT_FALSE(solution.solved);
  EXPECT_NE(solution.status.find("stage 3"), std::string::npos) << solution.status;
}
