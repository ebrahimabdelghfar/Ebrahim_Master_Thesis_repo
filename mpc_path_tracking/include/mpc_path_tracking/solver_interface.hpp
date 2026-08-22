#ifndef MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_
#define MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_

#include <memory>
#include <string>
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
  // Why the solve ended the way it did, in the backend's own terms
  // ("MAXITER", "INFEASIBLE", "non-finite QP data at stage 37", ...).
  // Without this every distinct failure mode surfaces as the same
  // "QP solve failed" fallback line and there is nothing to act on.
  std::string status{"OK"};
  int status_code{0};      // backend-native code (acados return_values_t)
  int iterations{0};       // qpOASES working-set recalculations / HPIPM iters
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

// Reports the same physically-meaningful cost (genuine non-negative squared
// tracking error, see SolverProblem::cost_offset) for any backend, computed
// directly from the solved trajectory rather than each backend's internal
// QP objective value - so OsqpMpcSolver and AcadosMpcSolver agree even
// though their internal objective scaling/augmentation differs.
double computeSolutionCost(const SolverProblem & problem, const SolverSolution & solution);

// Stage 2 backend: same SolverProblem/SolverSolution as OsqpMpcSolver, but
// solved via acados' linear-MPC QP interface (acados_c/ocp_qp_interface.h -
// no CasADi/Python codegen involved, this is the "OCP QP" C API, not
// "OCP NLP"). acados' per-stage general constraints and stage cost are
// strictly stage-local (only x_k, u_k - no cross-stage terms), so the
// u_k - u_{k-1} rate limit/penalty this controller needs is expressed via
// the standard augmented-state trick: x_aug = [x; u_prev] (nx_aug = 8),
// with Bd_aug = [Bd; I] so stage k+1's u_prev slot always equals u_k. See
// acados_mpc_solver.cpp for the exact per-stage matrix construction.
struct AcadosSolverSettings
{
  std::string qp_solver{"PARTIAL_CONDENSING_HPIPM"};
  int cond_N{5};          // partial-condensing block size (PARTIAL_CONDENSING_HPIPM only)
  // HPIPM interior-point iterations; qpOASES working-set recalculations
  // (max_nwsr) - the latter scales with the condensed problem size.
  int iter_max{1000};
  double tol_stat{1e-4};
  double tol_eq{1e-4};
  double tol_ineq{1e-4};
  double tol_comp{1e-4};
  bool warm_start{true};
  int print_level{0};
};

class AcadosMpcSolver : public SolverInterface
{
public:
  // N: prediction horizon steps (fixed for the node's lifetime, see
  // ParameterManager - horizon.N is not a runtime-mutable parameter), used
  // to size the acados QP structures once at construction.
  AcadosMpcSolver(int N, const AcadosSolverSettings & settings);
  ~AcadosMpcSolver() override;

  AcadosMpcSolver(const AcadosMpcSolver &) = delete;
  AcadosMpcSolver & operator=(const AcadosMpcSolver &) = delete;

  bool solve(const SolverProblem & problem, SolverSolution & solution) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__SOLVER_INTERFACE_HPP_
