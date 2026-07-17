#include "mpc_path_tracking/solver_interface.hpp"

#include <chrono>
#include <stdexcept>

#include <acados_c/ocp_qp_interface.h>

namespace mpc_path_tracking
{

namespace
{
// acados_c setters are declared with inconsistent const-ness across the API
// (ocp_qp_in_set takes `char *field`, the rest take `const char *`) -
// const_cast is safe either way since we only ever pass string literals.
inline char * fld(const char * s) {return const_cast<char *>(s);}

constexpr int kNx = 6;    // State: [X, Y, psi, vx, vy, r]
constexpr int kNu = 2;    // Input: [delta, a]
// Augmented state x_aug = [x; u_prev], nx_aug = 8. acados' per-stage
// general constraints/cost are stage-local (only x_k, u_k - no cross-stage
// terms), but this controller's rate limit and Rrate cost need
// u_k - u_{k-1}. Standard fix: carry u_{k-1} as extra state components so
// the coupling becomes an ordinary stage-local term - see solve() below.
constexpr int kNxAug = kNx + kNu;   // 8
}  // namespace

struct AcadosMpcSolver::Impl
{
  int N{0};
  AcadosSolverSettings settings;

  ocp_qp_xcond_solver_config * config{nullptr};
  ocp_qp_dims * dims{nullptr};
  ocp_qp_xcond_solver_dims * solver_dims{nullptr};
  void * opts{nullptr};
  ocp_qp_in * qp_in{nullptr};
  ocp_qp_out * qp_out{nullptr};
  ocp_qp_solver * solver{nullptr};

  ~Impl()
  {
    if (solver_dims) {ocp_qp_xcond_solver_dims_free(solver_dims);}
    if (dims) {ocp_qp_dims_free(dims);}
    if (config) {ocp_qp_xcond_solver_config_free(config);}
    if (opts) {ocp_qp_xcond_solver_opts_free(static_cast<ocp_qp_xcond_solver_opts *>(opts));}
    if (qp_in) {ocp_qp_in_free(qp_in);}
    if (qp_out) {ocp_qp_out_free(qp_out);}
    if (solver) {ocp_qp_solver_destroy(solver);}
  }
};

AcadosMpcSolver::AcadosMpcSolver(int N, const AcadosSolverSettings & settings)
: impl_(std::make_unique<Impl>())
{
  impl_->N = N;
  impl_->settings = settings;

  impl_->config = ocp_qp_xcond_solver_config_create_from_name(settings.qp_solver.c_str());
  if (!impl_->config) {
    throw std::runtime_error(
      "AcadosMpcSolver: unknown solver.acados.qp_solver '" + settings.qp_solver + "'");
  }

  impl_->dims = ocp_qp_dims_create(N);
  int nx_aug = kNxAug;
  int nu = kNu;
  int nu_terminal = 0;
  int nbx0 = kNxAug;
  int nbu = kNu;
  int ng = kNu;   // one general (rate) constraint row per control component
  int ng_terminal = 0;
  for (int i = 0; i <= N; ++i) {
    ocp_qp_dims_set(impl_->config, impl_->dims, i, "nx", &nx_aug);
    ocp_qp_dims_set(impl_->config, impl_->dims, i, "nu", &nu);
  }
  ocp_qp_dims_set(impl_->config, impl_->dims, 0, "nbx", &nbx0);
  for (int i = 0; i < N; ++i) {
    ocp_qp_dims_set(impl_->config, impl_->dims, i, "nbu", &nbu);
    ocp_qp_dims_set(impl_->config, impl_->dims, i, "ng", &ng);
  }
  ocp_qp_dims_set(impl_->config, impl_->dims, N, "nu", &nu_terminal);
  ocp_qp_dims_set(impl_->config, impl_->dims, N, "ng", &ng_terminal);

  impl_->qp_in = ocp_qp_in_create(impl_->dims);
  impl_->qp_out = ocp_qp_out_create(impl_->dims);

  impl_->solver_dims =
    ocp_qp_xcond_solver_dims_create_from_ocp_qp_dims(impl_->config, impl_->dims);
  impl_->opts = ocp_qp_xcond_solver_opts_create(impl_->config, impl_->solver_dims);
  auto * opts = static_cast<ocp_qp_xcond_solver_opts *>(impl_->opts);

  if (settings.qp_solver == "PARTIAL_CONDENSING_HPIPM") {
    int cond_N = settings.cond_N;
    ocp_qp_xcond_solver_opts_set(impl_->config, opts, "cond_N", &cond_N);
  }
  int iter_max = settings.iter_max;
  double tol_stat = settings.tol_stat;
  double tol_eq = settings.tol_eq;
  double tol_ineq = settings.tol_ineq;
  double tol_comp = settings.tol_comp;
  int warm_start = settings.warm_start ? 1 : 0;
  int print_level = settings.print_level;
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "iter_max", &iter_max);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "tol_stat", &tol_stat);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "tol_eq", &tol_eq);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "tol_ineq", &tol_ineq);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "tol_comp", &tol_comp);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "warm_start", &warm_start);
  ocp_qp_xcond_solver_opts_set(impl_->config, opts, "print_level", &print_level);

  impl_->solver = ocp_qp_create(impl_->config, impl_->solver_dims, impl_->opts);
}

AcadosMpcSolver::~AcadosMpcSolver() = default;

bool AcadosMpcSolver::solve(const SolverProblem & problem, SolverSolution & solution)
{
  const auto t_start = std::chrono::steady_clock::now();
  const int N = problem.N;
  auto * config = impl_->config;
  auto * qp_in = impl_->qp_in;

  // Fixed x_aug_0 = [x0; u_prev] - pins the augmented "previous control"
  // slot to the real previous command, so stage 0's rate constraint/cost
  // (below) is w.r.t. the actual last-applied input, exactly like OSQP's
  // special-cased k=0 rate row.
  Eigen::Matrix<double, kNxAug, 1> x_aug0;
  x_aug0.head<kNx>() = problem.x0;
  x_aug0.tail<kNu>() = problem.u_prev;
  int idxbx0[kNxAug];
  for (int i = 0; i < kNxAug; ++i) {idxbx0[i] = i;}
  ocp_qp_in_set(config, qp_in, 0, fld("idxbx"), idxbx0);
  ocp_qp_in_set(config, qp_in, 0, fld("lbx"), x_aug0.data());
  ocp_qp_in_set(config, qp_in, 0, fld("ubx"), x_aug0.data());

  int idxbu[kNu];
  for (int i = 0; i < kNu; ++i) {idxbu[i] = i;}

  for (int k = 0; k < N; ++k) {
    const MpcStage & stage = problem.stages[k];

    Eigen::Matrix<double, kNxAug, kNxAug> Ad_aug = Eigen::Matrix<double, kNxAug, kNxAug>::Zero();
    Ad_aug.block<kNx, kNx>(0, 0) = stage.Ad;

    Eigen::Matrix<double, kNxAug, kNu> Bd_aug = Eigen::Matrix<double, kNxAug, kNu>::Zero();
    Bd_aug.block<kNx, kNu>(0, 0) = stage.Bd;
    Bd_aug.block<kNu, kNu>(kNx, 0) = Eigen::Matrix<double, kNu, kNu>::Identity();

    Eigen::Matrix<double, kNxAug, 1> c_aug = Eigen::Matrix<double, kNxAug, 1>::Zero();
    c_aug.head<kNx>() = stage.c;

    // Stage cost: original state-error cost unchanged, plus the augmented
    // block reproducing (u_k - u_prev_k)^T Rrate (u_k - u_prev_k), where
    // u_prev_k is the augmented state's tail (see kNxAug comment above).
    // The 2x factor matches acados' 0.5*[x;u]^T H [x;u] convention exactly
    // the way OSQP's P=2Q already does (both need doubled weights to
    // reproduce the same physical cost value).
    Eigen::Matrix<double, kNxAug, kNxAug> Q_aug = Eigen::Matrix<double, kNxAug, kNxAug>::Zero();
    Q_aug.block<kNx, kNx>(0, 0) = 2.0 * stage.Qx;
    Q_aug.block<kNu, kNu>(kNx, kNx) = 2.0 * problem.Rrate;

    Eigen::Matrix<double, kNxAug, 1> q_aug = Eigen::Matrix<double, kNxAug, 1>::Zero();
    q_aug.head<kNx>() = 2.0 * stage.qx;

    Eigen::Matrix<double, kNu, kNu> R_aug = 2.0 * (problem.R + problem.Rrate);

    Eigen::Matrix<double, kNu, kNxAug> S_aug = Eigen::Matrix<double, kNu, kNxAug>::Zero();
    S_aug.block<kNu, kNu>(0, kNx) = -2.0 * problem.Rrate;

    Eigen::Matrix<double, kNu, 1> r_aug = Eigen::Matrix<double, kNu, 1>::Zero();

    // Hard rate constraint, expressed as the stage-local general
    // constraint lg <= D*u_k + C*x_aug_k <= ug, D=I, C=[0 | -I] (picks out
    // -u_prev_k), i.e. lg <= u_k - u_prev_k <= ug.
    Eigen::Matrix<double, kNu, kNxAug> C_aug = Eigen::Matrix<double, kNu, kNxAug>::Zero();
    C_aug.block<kNu, kNu>(0, kNx) = -Eigen::Matrix<double, kNu, kNu>::Identity();
    Eigen::Matrix<double, kNu, kNu> D_aug = Eigen::Matrix<double, kNu, kNu>::Identity();
    Eigen::Matrix<double, kNu, 1> lg = problem.u_rate_min * problem.dt;
    Eigen::Matrix<double, kNu, 1> ug = problem.u_rate_max * problem.dt;

    ocp_qp_in_set(config, qp_in, k, fld("A"), Ad_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("B"), Bd_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("b"), c_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("Q"), Q_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("S"), S_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("R"), R_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("q"), q_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("r"), r_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("C"), C_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("D"), D_aug.data());
    ocp_qp_in_set(config, qp_in, k, fld("lg"), lg.data());
    ocp_qp_in_set(config, qp_in, k, fld("ug"), ug.data());

    ocp_qp_in_set(config, qp_in, k, fld("idxbu"), idxbu);
    // problem.u_min/u_max are already Input (Eigen::Vector2d) - .data() is
    // fine to pass without a local copy since ocp_qp_in_set copies the
    // values into acados' own storage during the call.
    ocp_qp_in_set(config, qp_in, k, fld("lbu"), const_cast<double *>(problem.u_min.data()));
    ocp_qp_in_set(config, qp_in, k, fld("ubu"), const_cast<double *>(problem.u_max.data()));
  }

  // Terminal stage: state cost only, no controls (nu_e = 0).
  Eigen::Matrix<double, kNxAug, kNxAug> Q_terminal_aug =
    Eigen::Matrix<double, kNxAug, kNxAug>::Zero();
  Q_terminal_aug.block<kNx, kNx>(0, 0) = 2.0 * problem.Qx_terminal;
  Eigen::Matrix<double, kNxAug, 1> q_terminal_aug = Eigen::Matrix<double, kNxAug, 1>::Zero();
  q_terminal_aug.head<kNx>() = 2.0 * problem.qx_terminal;
  ocp_qp_in_set(config, qp_in, N, fld("Q"), Q_terminal_aug.data());
  ocp_qp_in_set(config, qp_in, N, fld("q"), q_terminal_aug.data());

  const int acados_status = ocp_qp_solve(impl_->solver, qp_in, impl_->qp_out);

  const auto t_end = std::chrono::steady_clock::now();
  solution.solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  if (acados_status != ACADOS_SUCCESS) {
    solution.solved = false;
    return false;
  }

  solution.x.resize(N + 1);
  solution.u.resize(N);
  for (int k = 0; k <= N; ++k) {
    Eigen::Matrix<double, kNxAug, 1> x_aug;
    ocp_qp_out_get(impl_->qp_out, k, "x", x_aug.data());
    solution.x[k] = x_aug.head<kNx>();
  }
  for (int k = 0; k < N; ++k) {
    Eigen::Matrix<double, kNu, 1> u_k;
    ocp_qp_out_get(impl_->qp_out, k, "u", u_k.data());
    solution.u[k] = u_k;
  }

  solution.cost = computeSolutionCost(problem, solution);
  solution.solved = true;
  return true;
}

}  // namespace mpc_path_tracking
