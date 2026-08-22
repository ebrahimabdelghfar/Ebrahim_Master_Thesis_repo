#include "mpc_path_tracking/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace mpc_path_tracking
{

MpcController::MpcController(
  const VehicleModel & model, std::unique_ptr<SolverInterface> solver,
  const MpcConfig & config)
: model_(model), solver_(std::move(solver)), config_(config)
{
}

double MpcController::computeAdaptiveDt(const ReferencePoint & nearest) const
{
  const double v_floor = 0.5;
  const double v_ref = std::max(nearest.vx, v_floor);
  const double dt = config_.horizon_distance_m / (static_cast<double>(config_.N) * v_ref);
  return std::clamp(dt, config_.dt_min, config_.dt_max);
}

VehicleModel::SteadyState MpcController::referenceEquilibrium(const ReferencePoint & ref) const
{
  return model_.steadyStateCornering(ref.vx, ref.kappa);
}

void MpcController::equilibriumPoint(
  const ReferencePoint & ref, const VehicleModel::SteadyState & ss,
  State & x_lin, Input & u_lin) const
{
  x_lin << ref.x, ref.y, ref.psi - ss.beta,
    std::max(ref.vx, VehicleModel::kVxFloor), ss.vy, ref.vx * ref.kappa;
  u_lin << ss.delta, 0.0;
}

void MpcController::errorCostMatrices(
  const ReferencePoint & ref, const VehicleModel::SteadyState & ss,
  const Eigen::Matrix<double, 5, 1> & Qdiag,
  StateJacobian & Qx, State & qx, double & cost_offset) const
{
  Eigen::Matrix<double, 5, 6> C = Eigen::Matrix<double, 5, 6>::Zero();
  Eigen::Matrix<double, 5, 1> r;

  const double s = std::sin(ref.psi);
  const double c = std::cos(ref.psi);

  C(0, 0) = -s;
  C(0, 1) = c;
  r(0) = -s * ref.x + c * ref.y;

  // For the CG velocity to lie along the path the BODY must point beta off
  // the path tangent, since the velocity vector sits at psi + beta. Targeting
  // the tangent itself asks the car to be crabbing the wrong way through
  // every corner; harmless when beta ~ 0 (low speed), a growing conflict with
  // the lateral-error term as speed rises.
  C(1, 2) = 1.0;
  r(1) = ref.psi - ss.beta;

  C(2, 3) = 1.0;
  r(2) = ref.vx;

  // The sideslip the turn actually requires - NOT zero. Demanding vy = 0
  // while also demanding the path is asking for two mutually exclusive
  // states, and the QP resolves the conflict differently from cycle to
  // cycle, which is what the steering chatter above ~15 m/s was.
  C(3, 4) = 1.0;
  r(3) = ss.vy;

  C(4, 5) = 1.0;
  r(4) = ref.vx * ref.kappa;

  const Eigen::Matrix<double, 5, 5> Qdiag_mat = Qdiag.asDiagonal();
  Qx = C.transpose() * Qdiag_mat * C;
  qx = -C.transpose() * Qdiag_mat * r;
  cost_offset = r.transpose() * Qdiag_mat * r;
}

std::string MpcController::diagnoseHorizon(
  const SolverProblem & problem, const std::vector<ReferencePoint> & horizon) const
{
  int worst_k = -1;
  double worst_rho = 0.0;
  for (int k = 0; k < problem.N; ++k) {
    const StateJacobian & Ad = problem.stages[k].Ad;
    if (!Ad.allFinite()) {
      return " | stage " + std::to_string(k) + " Jacobian is non-finite";
    }
    // Eigenvalues only - no eigenvectors - of a 6x6, on the failure path
    // only, so the cost is irrelevant next to the failed solve itself.
    const double rho = Eigen::EigenSolver<StateJacobian>(Ad, false)
      .eigenvalues().cwiseAbs().maxCoeff();
    if (rho > worst_rho) {
      worst_rho = rho;
      worst_k = k;
    }
  }
  if (worst_k < 0) {
    return "";
  }
  char buf[220];
  std::snprintf(
    buf, sizeof(buf),
    " | worst stage k=%d: rho(Ad)=%.3f at v_ref=%.1f m/s, kappa=%.4f 1/m, dt=%.3f s "
    "-> prediction grows 1e%+.1f over N=%d",
    worst_k, worst_rho, horizon[worst_k].vx, horizon[worst_k].kappa, problem.dt,
    problem.N * std::log10(std::max(worst_rho, 1e-12)), problem.N);
  return buf;
}

MpcStage MpcController::buildStage(
  const ReferencePoint & ref_k, const VehicleModel::SteadyState & ss,
  const State & x_lin, const Input & u_lin, double dt,
  const Eigen::Matrix<double, 5, 1> & Qdiag) const
{
  MpcStage stage;
  model_.linearizeDiscrete(x_lin, u_lin, dt, stage.Ad, stage.Bd, stage.c);
  errorCostMatrices(ref_k, ss, Qdiag, stage.Qx, stage.qx, stage.cost_offset);
  return stage;
}

MpcOutput MpcController::computeCommand(
  const State & x0, const Input & u_prev, const ReferenceTrajectoryHandler & ref_handler)
{
  MpcOutput out;
  if (!ref_handler.hasWaypoints()) {
    out.status = "no waypoints";
    return out;
  }

  const ReferencePoint nearest = ref_handler.nearestPoint(x0(0), x0(1));
  const double dt = computeAdaptiveDt(nearest);
  const auto horizon = ref_handler.buildHorizon(x0(0), x0(1), x0(2), config_.N, dt);
  if (static_cast<int>(horizon.size()) != config_.N + 1) {
    out.status = "short reference horizon (" + std::to_string(horizon.size()) + "/" +
      std::to_string(config_.N + 1) + " points)";
    return out;
  }

  // NOTE: successive linearization about the previous solution (the standard
  // LTV-MPC / real-time-iteration scheme) was implemented here and MEASURED TO
  // BE HARMFUL for this problem, so it is deliberately not used.
  //
  // Closed-loop harness, traj_race_cl.csv, 27 m/s, 20 Hz, 1 tick of transport
  // delay: linearizing about the reference equilibrium gave 0 solve failures at
  // ~15 ms; linearizing about the previous solution gave 102 failures out of
  // 600 cycles at ~90 ms. The diagnostics say why - the previous solution's far
  // stages are not near any equilibrium, and this tire set is stiff enough that
  // the Jacobian there has rho(Ad) up to 2.1e6 (vs ~1.0 at the equilibrium).
  // Over N=100 with full condensing that is unsolvable.
  //
  // The reference equilibrium, by construction, always linearizes at a point
  // where the lateral dynamics are balanced, which is what keeps every stage
  // contractive. Revisit only alongside a shorter horizon or partial
  // condensing. prev_states_/prev_inputs_ are still maintained for warm start.
  out.relinearized = false;

  SolverProblem problem;
  problem.N = config_.N;
  problem.dt = dt;
  problem.x0 = x0;
  problem.stages.reserve(config_.N);
  problem.cost_offset = 0.0;
  for (int k = 0; k < config_.N; ++k) {
    // The cost target is always the reference equilibrium; only the point the
    // dynamics are linearized about comes from the previous solution.
    const VehicleModel::SteadyState ss = referenceEquilibrium(horizon[k]);
    if (!ss.converged) {
      ++out.infeasible_ref_stages;
    }
    State x_lin;
    Input u_lin;
    equilibriumPoint(horizon[k], ss, x_lin, u_lin);
    problem.stages.push_back(buildStage(horizon[k], ss, x_lin, u_lin, dt, config_.cost.Q));
    problem.cost_offset += problem.stages.back().cost_offset;
  }
  double terminal_cost_offset = 0.0;
  const VehicleModel::SteadyState ss_terminal = referenceEquilibrium(horizon[config_.N]);
  if (!ss_terminal.converged) {
    ++out.infeasible_ref_stages;
  }
  errorCostMatrices(
    horizon[config_.N], ss_terminal, config_.cost.Qf, problem.Qx_terminal, problem.qx_terminal,
    terminal_cost_offset);
  problem.cost_offset += terminal_cost_offset;
  // k=0 rate-penalty term is (u_0 - u_prev)^T Rrate (u_0 - u_prev); the
  // solver only builds the quadratic-in-z and linear-in-z parts, so the
  // u_prev^T Rrate u_prev constant is added back here too.
  problem.cost_offset += u_prev.transpose() * config_.cost.Rrate.asDiagonal() * u_prev;

  problem.R = config_.cost.R.asDiagonal();
  problem.Rrate = config_.cost.Rrate.asDiagonal();
  problem.u_prev = u_prev;
  problem.u_min = Input(config_.limits.steering_min, config_.limits.accel_min);
  problem.u_max = Input(config_.limits.steering_max, config_.limits.accel_max);
  problem.u_rate_min = Input(-config_.limits.steering_rate_max, -config_.limits.jerk_max);
  problem.u_rate_max = Input(config_.limits.steering_rate_max, config_.limits.jerk_max);

  SolverSolution solution;
  const bool ok = solver_->solve(problem, solution);

  out.solved = ok && solution.solved;
  out.dt_used = dt;
  if (!out.solved) {
    out.solve_time_ms = solution.solve_time_ms;
    out.status = solution.status + diagnoseHorizon(problem, horizon);
    // Never linearize the next cycle about a solution that failed - fall back
    // to the reference equilibrium, which is always well defined.
    prev_states_.clear();
    prev_inputs_.clear();
  }
  if (out.solved) {
    out.u0 = solution.u.front();
    out.predicted_states = solution.x;
    out.predicted_inputs = solution.u;
    out.solve_time_ms = solution.solve_time_ms;
    out.cost = solution.cost;
    prev_states_ = solution.x;
    prev_inputs_ = solution.u;
  }
  return out;
}

}  // namespace mpc_path_tracking
