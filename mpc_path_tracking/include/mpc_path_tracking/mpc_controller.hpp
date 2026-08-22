#ifndef MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_
#define MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "mpc_path_tracking/reference_trajectory_handler.hpp"
#include "mpc_path_tracking/solver_interface.hpp"
#include "mpc_path_tracking/vehicle_model.hpp"

namespace mpc_path_tracking
{

// Diagonal weights over the path-relative error output
// [e_y, e_psi, vx_err, vy, r_err].
struct MpcCostWeights
{
  Eigen::Matrix<double, 5, 1> Q;
  Eigen::Matrix<double, 5, 1> Qf;
  Eigen::Vector2d R;
  Eigen::Vector2d Rrate;
};

struct MpcLimits
{
  double steering_min{-0.4189};
  double steering_max{0.4189};
  double steering_rate_max{3.2};
  double accel_min{-8.26};
  double accel_max{7.51};
  double jerk_max{1000.0};   // effectively unconstrained unless tuned
};

struct MpcConfig
{
  int N{20};
  double dt_min{0.02};
  double dt_max{0.08};
  double horizon_distance_m{8.0};
  MpcCostWeights cost;
  MpcLimits limits;
};

struct MpcOutput
{
  bool solved{false};
  Input u0{Input::Zero()};
  std::vector<State> predicted_states;
  std::vector<Input> predicted_inputs;
  double solve_time_ms{0.0};
  double cost{0.0};
  double dt_used{0.0};
  // Horizon stages whose reference (vx, kappa) has no steady-state solution
  // for the current tire params - the raceline asks for more grip than the
  // model has there. Non-zero means the reference is partly infeasible.
  int infeasible_ref_stages{0};
  // True when the horizon was linearized about the previous solution
  // (successive linearization); false on the first solve and after a failure,
  // where it falls back to the reference equilibrium.
  bool relinearized{false};
  // On failure: the backend's own status plus, when the linearized model is
  // the culprit, the worst stage's spectral radius and how far the
  // prediction diverges over the horizon (see diagnoseHorizon).
  std::string status{"OK"};
};

// Builds and solves one LTV-MPC QP per control cycle over the nonlinear
// dynamic-bicycle + Pacejka model.
//
// The reference is turned into a DYNAMICALLY FEASIBLE operating point via
// VehicleModel::steadyStateCornering before it is used, so vy_ref and the
// heading target account for the sideslip the car must actually carry through
// a corner, and every stage is linearized at a point where the lateral
// dynamics are balanced.
//
// Each stage is linearized about that reference equilibrium, NOT about the
// previous solution. Successive linearization is the textbook choice and was
// tried here; it was measured to be much worse for this problem (see the note
// in computeCommand). The equilibrium keeps every stage Jacobian contractive,
// which is what a 100-stage fully-condensed QP over a stiff tire model needs.
class MpcController
{
public:
  MpcController(
    const VehicleModel & model, std::unique_ptr<SolverInterface> solver,
    const MpcConfig & config);

  MpcOutput computeCommand(
    const State & x0, const Input & u_prev,
    const ReferenceTrajectoryHandler & ref_handler);

  // Both of these invalidate the cached previous solution: it is only a valid
  // linearization trajectory for the model and horizon that produced it.
  void setConfig(const MpcConfig & config)
  {
    config_ = config;
    prev_states_.clear();
    prev_inputs_.clear();
  }
  const MpcConfig & config() const {return config_;}

  // Forwards to the internal VehicleModel copy actually used by
  // computeCommand()/buildStage() - this is the instance the QP solve
  // reads, not whatever VehicleModel the caller originally constructed
  // MpcController with.
  void setTireParams(const TireParams & tire)
  {
    model_.setTireParams(tire);
    prev_states_.clear();
    prev_inputs_.clear();
  }

  // The tire params computeCommand() actually solves against. The ROS
  // parameters are NOT updated by mpc/update_params, so this is the only
  // truthful source once sysid has pushed an identification.
  TireParams effectiveTireParams() const {return model_.tireParams();}

  // Rolls `x` forward by `dt` under the same model the solver uses. Callers
  // must go through this rather than their own VehicleModel instance: only
  // model_ receives the tire updates from mpc/update_params, so any other
  // copy silently predicts with the startup params forever.
  State predictState(const State & x, const Input & u, double dt) const
  {
    return model_.integrateRk4(x, u, dt);
  }

private:
  double computeAdaptiveDt(const ReferencePoint & nearest) const;

  // The dynamically feasible operating point for `ref`: the state the vehicle
  // must actually be in to hold this speed and curvature, including the
  // sideslip and the heading offset from the path tangent, plus the steering
  // angle that sustains it. This is both the cost target and the fallback
  // linearization point.
  VehicleModel::SteadyState referenceEquilibrium(const ReferencePoint & ref) const;
  void equilibriumPoint(
    const ReferencePoint & ref, const VehicleModel::SteadyState & ss,
    State & x_lin, Input & u_lin) const;

  MpcStage buildStage(
    const ReferencePoint & ref_k, const VehicleModel::SteadyState & ss,
    const State & x_lin, const Input & u_lin, double dt,
    const Eigen::Matrix<double, 5, 1> & Qdiag) const;
  void errorCostMatrices(
    const ReferencePoint & ref, const VehicleModel::SteadyState & ss,
    const Eigen::Matrix<double, 5, 1> & Qdiag,
    StateJacobian & Qx, State & qx, double & cost_offset) const;

  // Failure-path only: finds the stage whose discrete-time Jacobian has the
  // largest spectral radius and reports it together with the reference speed
  // there and the resulting rho^N growth over the horizon. An open-loop
  // unstable linearization (oversteering tire set, too-coarse dt) makes the
  // condensed QP span tens of orders of magnitude, which every dense QP
  // solver reports as a generic failure with no hint at the real cause.
  std::string diagnoseHorizon(
    const SolverProblem & problem, const std::vector<ReferencePoint> & horizon) const;

  VehicleModel model_;
  std::unique_ptr<SolverInterface> solver_;
  MpcConfig config_;

  // Previous cycle's predicted trajectory, used (shifted by one) as this
  // cycle's linearization points. Cleared on any solve failure so a bad
  // solution is never propagated into the next linearization.
  std::vector<State> prev_states_;
  std::vector<Input> prev_inputs_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_
