#ifndef MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_
#define MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_

#include <memory>
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
};

// Builds and solves one LTV-MPC QP per control cycle: linearizes the
// nonlinear dynamic-bicycle+Pacejka model about the reference trajectory
// (not about the previous solution), which avoids needing a warm-started
// nominal trajectory on the very first solve and keeps every solve
// self-contained.
class MpcController
{
public:
  MpcController(
    const VehicleModel & model, std::unique_ptr<SolverInterface> solver,
    const MpcConfig & config);

  MpcOutput computeCommand(
    const State & x0, const Input & u_prev,
    const ReferenceTrajectoryHandler & ref_handler);

  void setConfig(const MpcConfig & config) {config_ = config;}
  const MpcConfig & config() const {return config_;}

private:
  double computeAdaptiveDt(const ReferencePoint & nearest) const;
  MpcStage buildStage(
    const ReferencePoint & ref_k, double dt, const Eigen::Matrix<double, 5, 1> & Qdiag) const;
  void errorCostMatrices(
    const ReferencePoint & ref, const Eigen::Matrix<double, 5, 1> & Qdiag,
    StateJacobian & Qx, State & qx) const;

  VehicleModel model_;
  std::unique_ptr<SolverInterface> solver_;
  MpcConfig config_;
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__MPC_CONTROLLER_HPP_
