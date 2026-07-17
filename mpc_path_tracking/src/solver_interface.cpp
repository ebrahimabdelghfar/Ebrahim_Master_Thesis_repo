#include "mpc_path_tracking/solver_interface.hpp"

#include <chrono>

#include <OsqpEigen/OsqpEigen.h>

namespace mpc_path_tracking
{

namespace
{
// Adds M (r x c) into the triplet list at (row0, col0). Duplicate (row,
// col) entries across calls are summed by Eigen::SparseMatrix::setFromTriplets,
// which is exactly the accumulation the cost-assembly below relies on.
template<typename Derived>
void addBlock(
  std::vector<Eigen::Triplet<double>> & triplets, int row0, int col0,
  const Eigen::MatrixBase<Derived> & M)
{
  for (int i = 0; i < M.rows(); ++i) {
    for (int j = 0; j < M.cols(); ++j) {
      if (M(i, j) != 0.0) {
        triplets.emplace_back(row0 + i, col0 + j, M(i, j));
      }
    }
  }
}
}  // namespace

double computeSolutionCost(const SolverProblem & problem, const SolverSolution & solution)
{
  // Reconstructs the same physical (non-negative, offset-corrected) cost
  // that OsqpMpcSolver's "0.5*z^T P z + q^T z + cost_offset" formula
  // produces, but directly from the solved trajectory instead of that
  // flat-z quadratic form - so any backend reports an identical value as
  // long as it returns the same solution.x/solution.u. Verified equivalent
  // term-by-term against the P/q assembly above (Qx/qx blocks -> 0.5*2Qx +
  // 2qx per stage; R blocks -> 0.5*2R per stage; the banded Rrate
  // cross-blocks between u_k and u_{k-1} collapse to the standard
  // telescoping quadratic-difference identity sum_k (u_k-u_{k-1})^T Rrate
  // (u_k-u_{k-1})).
  const int N = problem.N;
  double raw = 0.0;
  for (int k = 0; k <= N; ++k) {
    const StateJacobian & Qx = (k < N) ? problem.stages[k].Qx : problem.Qx_terminal;
    const State & qx = (k < N) ? problem.stages[k].qx : problem.qx_terminal;
    raw += solution.x[k].dot(Qx * solution.x[k]) + 2.0 * qx.dot(solution.x[k]);
  }
  Input u_prev = problem.u_prev;
  for (int k = 0; k < N; ++k) {
    raw += solution.u[k].dot(problem.R * solution.u[k]);
    const Input du = solution.u[k] - u_prev;
    raw += du.dot(problem.Rrate * du);
    u_prev = solution.u[k];
  }
  return raw + problem.cost_offset;
}

OsqpMpcSolver::OsqpMpcSolver(const OsqpSolverSettings & settings)
: settings_(settings)
{
}

bool OsqpMpcSolver::solve(const SolverProblem & problem, SolverSolution & solution)
{
  const auto t_start = std::chrono::steady_clock::now();

  const int N = problem.N;
  constexpr int nx = 6;
  constexpr int nu = 2;
  const int n_x_total = nx * (N + 1);
  const int n_u_total = nu * N;
  const int n_z = n_x_total + n_u_total;

  auto xIdx = [&](int k) {return nx * k;};
  auto uIdx = [&](int k) {return n_x_total + nu * k;};

  // ---- Cost: 0.5 z^T P z + q^T z ----
  std::vector<Eigen::Triplet<double>> p_triplets;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n_z);

  for (int k = 0; k <= N; ++k) {
    const StateJacobian & Qx = (k < N) ? problem.stages[k].Qx : problem.Qx_terminal;
    const State & qx = (k < N) ? problem.stages[k].qx : problem.qx_terminal;
    addBlock(p_triplets, xIdx(k), xIdx(k), (2.0 * Qx).eval());
    q.segment<nx>(xIdx(k)) += 2.0 * qx;
  }

  for (int k = 0; k < N; ++k) {
    addBlock(p_triplets, uIdx(k), uIdx(k), (2.0 * problem.R).eval());
  }

  // Input-rate penalty, u_{-1} := problem.u_prev (parameter, not a variable).
  addBlock(p_triplets, uIdx(0), uIdx(0), (2.0 * problem.Rrate).eval());
  q.segment<nu>(uIdx(0)) += -2.0 * problem.Rrate * problem.u_prev;
  for (int k = 1; k < N; ++k) {
    addBlock(p_triplets, uIdx(k), uIdx(k), (2.0 * problem.Rrate).eval());
    addBlock(p_triplets, uIdx(k - 1), uIdx(k - 1), (2.0 * problem.Rrate).eval());
    addBlock(p_triplets, uIdx(k), uIdx(k - 1), (-2.0 * problem.Rrate).eval());
    addBlock(p_triplets, uIdx(k - 1), uIdx(k), (-2.0 * problem.Rrate).eval());
  }

  Eigen::SparseMatrix<double> P(n_z, n_z);
  P.setFromTriplets(p_triplets.begin(), p_triplets.end());
  P.makeCompressed();

  // ---- Constraints: l <= A z <= u ----
  const int n_eq = nx * (N + 1);
  const int n_ineq = 2 * nu * N;
  const int n_rows = n_eq + n_ineq;

  std::vector<Eigen::Triplet<double>> a_triplets;
  Eigen::VectorXd l = Eigen::VectorXd::Zero(n_rows);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(n_rows);

  int row = 0;
  // x_0 == problem.x0
  addBlock(a_triplets, row, xIdx(0), Eigen::Matrix<double, nx, nx>::Identity());
  l.segment<nx>(row) = problem.x0;
  u.segment<nx>(row) = problem.x0;
  row += nx;

  // x_{k+1} - Ad_k x_k - Bd_k u_k == c_k
  for (int k = 0; k < N; ++k) {
    addBlock(a_triplets, row, xIdx(k + 1), Eigen::Matrix<double, nx, nx>::Identity());
    addBlock(a_triplets, row, xIdx(k), (-problem.stages[k].Ad).eval());
    addBlock(a_triplets, row, uIdx(k), (-problem.stages[k].Bd).eval());
    l.segment<nx>(row) = problem.stages[k].c;
    u.segment<nx>(row) = problem.stages[k].c;
    row += nx;
  }

  // u_min <= u_k <= u_max
  for (int k = 0; k < N; ++k) {
    addBlock(a_triplets, row, uIdx(k), Eigen::Matrix<double, nu, nu>::Identity());
    l.segment<nu>(row) = problem.u_min;
    u.segment<nu>(row) = problem.u_max;
    row += nu;
  }

  // Rate limits: u_rate_min*dt <= u_k - u_{k-1} <= u_rate_max*dt (u_{-1} = u_prev)
  addBlock(a_triplets, row, uIdx(0), Eigen::Matrix<double, nu, nu>::Identity());
  l.segment<nu>(row) = problem.u_prev + problem.u_rate_min * problem.dt;
  u.segment<nu>(row) = problem.u_prev + problem.u_rate_max * problem.dt;
  row += nu;
  for (int k = 1; k < N; ++k) {
    addBlock(a_triplets, row, uIdx(k), Eigen::Matrix<double, nu, nu>::Identity());
    addBlock(a_triplets, row, uIdx(k - 1), (-Eigen::Matrix<double, nu, nu>::Identity()).eval());
    l.segment<nu>(row) = problem.u_rate_min * problem.dt;
    u.segment<nu>(row) = problem.u_rate_max * problem.dt;
    row += nu;
  }

  Eigen::SparseMatrix<double> A(n_rows, n_z);
  A.setFromTriplets(a_triplets.begin(), a_triplets.end());
  A.makeCompressed();

  // ---- Solve ----
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(settings_.warm_start);
  solver.settings()->setMaxIteration(settings_.max_iter);
  solver.settings()->setAbsoluteTolerance(settings_.eps_abs);
  solver.settings()->setRelativeTolerance(settings_.eps_rel);
  solver.settings()->setPolish(settings_.polish);
  solver.settings()->setTimeLimit(settings_.time_limit_s);

  solver.data()->setNumberOfVariables(n_z);
  solver.data()->setNumberOfConstraints(n_rows);
  if (!solver.data()->setHessianMatrix(P)) {return false;}
  if (!solver.data()->setGradient(q)) {return false;}
  if (!solver.data()->setLinearConstraintsMatrix(A)) {return false;}
  if (!solver.data()->setLowerBound(l)) {return false;}
  if (!solver.data()->setUpperBound(u)) {return false;}

  if (!solver.initSolver()) {
    solution.solved = false;
    return false;
  }

  const bool ok = solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
  const auto t_end = std::chrono::steady_clock::now();
  solution.solve_time_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (!ok) {
    solution.solved = false;
    return false;
  }

  const Eigen::VectorXd z = solver.getSolution();
  solution.x.resize(N + 1);
  solution.u.resize(N);
  for (int k = 0; k <= N; ++k) {
    solution.x[k] = z.segment<nx>(xIdx(k));
  }
  for (int k = 0; k < N; ++k) {
    solution.u[k] = z.segment<nu>(uIdx(k));
  }
  solution.cost = computeSolutionCost(problem, solution);
  solution.solved = true;
  return true;
}

}  // namespace mpc_path_tracking
