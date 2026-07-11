#include "mpc_path_tracking/mpc_controller.hpp"

#include <algorithm>
#include <cmath>

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

void MpcController::errorCostMatrices(
  const ReferencePoint & ref, const Eigen::Matrix<double, 5, 1> & Qdiag,
  StateJacobian & Qx, State & qx, double & cost_offset) const
{
  Eigen::Matrix<double, 5, 6> C = Eigen::Matrix<double, 5, 6>::Zero();
  Eigen::Matrix<double, 5, 1> r;

  const double s = std::sin(ref.psi);
  const double c = std::cos(ref.psi);

  C(0, 0) = -s;
  C(0, 1) = c;
  r(0) = -s * ref.x + c * ref.y;

  C(1, 2) = 1.0;
  r(1) = ref.psi;

  C(2, 3) = 1.0;
  r(2) = ref.vx;

  C(3, 4) = 1.0;
  r(3) = 0.0;

  C(4, 5) = 1.0;
  r(4) = ref.vx * ref.kappa;

  const Eigen::Matrix<double, 5, 5> Qdiag_mat = Qdiag.asDiagonal();
  Qx = C.transpose() * Qdiag_mat * C;
  qx = -C.transpose() * Qdiag_mat * r;
  cost_offset = r.transpose() * Qdiag_mat * r;
}

MpcStage MpcController::buildStage(
  const ReferencePoint & ref_k, double dt, const Eigen::Matrix<double, 5, 1> & Qdiag) const
{
  const double wheelbase = model_.vehicleParams().l_f + model_.vehicleParams().l_r;

  State x_lin;
  x_lin << ref_k.x, ref_k.y, ref_k.psi, std::max(ref_k.vx, VehicleModel::kVxFloor), 0.0,
    ref_k.vx * ref_k.kappa;
  Input u_lin;
  u_lin << std::atan(wheelbase * ref_k.kappa), 0.0;

  MpcStage stage;
  model_.linearizeDiscrete(x_lin, u_lin, dt, stage.Ad, stage.Bd, stage.c);
  errorCostMatrices(ref_k, Qdiag, stage.Qx, stage.qx, stage.cost_offset);
  return stage;
}

MpcOutput MpcController::computeCommand(
  const State & x0, const Input & u_prev, const ReferenceTrajectoryHandler & ref_handler)
{
  MpcOutput out;
  if (!ref_handler.hasWaypoints()) {
    return out;
  }

  const ReferencePoint nearest = ref_handler.nearestPoint(x0(0), x0(1));
  const double dt = computeAdaptiveDt(nearest);
  const auto horizon = ref_handler.buildHorizon(x0(0), x0(1), x0(2), config_.N, dt);
  if (static_cast<int>(horizon.size()) != config_.N + 1) {
    return out;
  }

  SolverProblem problem;
  problem.N = config_.N;
  problem.dt = dt;
  problem.x0 = x0;
  problem.stages.reserve(config_.N);
  problem.cost_offset = 0.0;
  for (int k = 0; k < config_.N; ++k) {
    problem.stages.push_back(buildStage(horizon[k], dt, config_.cost.Q));
    problem.cost_offset += problem.stages.back().cost_offset;
  }
  double terminal_cost_offset = 0.0;
  errorCostMatrices(
    horizon[config_.N], config_.cost.Qf, problem.Qx_terminal, problem.qx_terminal,
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
  if (out.solved) {
    out.u0 = solution.u.front();
    out.predicted_states = solution.x;
    out.predicted_inputs = solution.u;
    out.solve_time_ms = solution.solve_time_ms;
    out.cost = solution.cost;
  }
  return out;
}

}  // namespace mpc_path_tracking
