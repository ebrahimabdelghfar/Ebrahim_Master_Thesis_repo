#ifndef MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_
#define MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_

#include <vector>

#include "mpc_path_tracking/vehicle_model.hpp"

namespace mpc_path_tracking
{

// Per-stage (k = 0..N-1) LTV data: the affine dynamics x_{k+1} = Ad*x_k +
// Bd*u_k + c, and the quadratic state-cost data Qx = C_k^T*diag(Q)*C_k,
// qx = -C_k^T*diag(Q)*target_k, where C_k maps the full state to the
// path-relative error vector [e_y, e_psi, vx_err, vy, r_err] at stage k
// (see mpc_controller.cpp for the construction of C_k/target_k).
struct MpcStage
{
  StateJacobian Ad;
  InputJacobian Bd;
  State c;
  StateJacobian Qx;
  State qx;
  // target_k^T * diag(Q) * target_k - the "complete the square" constant
  // dropped by Qx/qx alone (see SolverProblem::cost_offset).
  double cost_offset{0.0};
};

// Full LTV-MPC problem for one control-cycle solve: N stages of dynamics
// plus one terminal cost stage, all in the stacked-state-and-input
// (sparse KKT) formulation.
struct SolverProblem
{
  int N{0};
  double dt{0.0};
  State x0;
  std::vector<MpcStage> stages;   // size N
  StateJacobian Qx_terminal;
  State qx_terminal;
  // Sum of every dropped "complete the square" constant (stage + terminal
  // target_k^T diag(Q) target_k, plus u_prev^T Rrate u_prev from the k=0
  // rate-penalty term) - added back into SolverSolution::cost so it reports
  // genuine non-negative squared tracking error instead of an internal QP
  // objective offset by however large the absolute-frame reference
  // happens to be (caught via inspecting a suspiciously large negative
  // solve cost in practice - see docs).
  double cost_offset{0.0};
  Eigen::Matrix2d R;
  Eigen::Matrix2d Rrate;
  Input u_prev;
  Input u_min;
  Input u_max;
  Input u_rate_min;   // rad/s or m/s^2, per-second
  Input u_rate_max;
};

struct SolverSolution
{
  bool solved{false};
  std::vector<Input> u;    // size N
  std::vector<State> x;    // size N+1
  double cost{0.0};
  double solve_time_ms{0.0};
};

class SolverInterface
{
public:
  virtual ~SolverInterface() = default;
  virtual bool solve(const SolverProblem & problem, SolverSolution & solution) = 0;
};

// Stage 1 backend: builds the sparse stacked-state-and-input QP
//   min  sum_k [x_k; u_k]^T-weighted quadratic cost
//   s.t. x_0 fixed, x_{k+1} = Ad_k*x_k + Bd_k*u_k + c_k, box + rate limits
// and solves it with OSQP via osqp-eigen. A future Stage 2 NMPC backend
// (e.g. ACADOS) implements the same SolverInterface without touching
// mpc_controller.cpp.
struct OsqpSolverSettings
{
  int max_iter{200};
  double eps_abs{1e-4};
  double eps_rel{1e-4};
  bool warm_start{true};
  bool polish{true};
  double time_limit_s{0.015};
};

class OsqpMpcSolver : public SolverInterface
{
public:
  explicit OsqpMpcSolver(const OsqpSolverSettings & settings);
  bool solve(const SolverProblem & problem, SolverSolution & solution) override;

private:
  OsqpSolverSettings settings_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_
