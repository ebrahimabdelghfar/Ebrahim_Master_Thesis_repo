#ifndef MPC_PATH_TRACKING__VEHICLE_MODEL_HPP_
#define MPC_PATH_TRACKING__VEHICLE_MODEL_HPP_

#include <mutex>

#include <Eigen/Dense>

namespace mpc_path_tracking
{

// State order: [X, Y, psi, vx, vy, r]
using State = Eigen::Matrix<double, 6, 1>;
// Input order: [delta (steering angle, rad), a (longitudinal accel, m/s^2)]
using Input = Eigen::Matrix<double, 2, 1>;
using StateJacobian = Eigen::Matrix<double, 6, 6>;
using InputJacobian = Eigen::Matrix<double, 6, 2>;

struct VehicleParams
{
  double mass{3.54};
  double Iz{0.05797};
  double l_f{0.162};
  double l_r{0.145};
  double h_cg{0.02};
};

// Per-axle Magic Formula / Pacejka coefficients [B, C, D, E].
// D is a dimensionless peak-friction-like coefficient; it multiplies the
// axle normal load Fz to produce a force, it is not a force by itself.
struct TireParams
{
  double Bf{2.4128};
  double Cf{4.8155};
  double Df{0.5922};
  double Ef{5.0};
  double Br{14.4445};
  double Cr{1.2129};
  double Dr{0.6842};
  double Er{0.8526};
};

// Dynamic single-track (bicycle) vehicle model with a Magic Formula /
// Pacejka tire model. Longitudinal tire force is not modeled per-axle;
// the input `a` is treated as the net longitudinal acceleration applied
// at the CG (consistent with the simulator's speed-tracking drivetrain
// model, which pure_pursuit already relies on for its `drive.speed`
// command).
class VehicleModel
{
public:
  VehicleModel(const VehicleParams & vehicle, const TireParams & tire);

  // MpcController keeps its own VehicleModel copy (see mpc_controller.hpp's
  // `model_` member) - tire_mutex_ is per-instance state, not shared data,
  // so copies must NOT copy the mutex itself, only the (thread-safely read)
  // vehicle_/tire_ values, each into a freshly-constructed mutex.
  VehicleModel(const VehicleModel & other);
  VehicleModel & operator=(const VehicleModel & other);

  // Continuous-time nonlinear dynamics xdot = f(x, u).
  State continuousDynamics(const State & x, const Input & u) const;

  // RK4 integration of the nonlinear dynamics over one step of size dt,
  // internally sub-stepped so that explicit RK4 stays inside its stability
  // region for the current tire stiffness and speed (see
  // integrationSubsteps).
  State integrateRk4(const State & x, const Input & u, double dt) const;

  // Number of RK4 sub-steps integrateRk4 splits `dt` into, from the fastest
  // lateral mode implied by the current tire params, mass/Iz and vx.
  int integrationSubsteps(const State & x, double dt) const;

  // Discrete-time affine LTV approximation about (x, u) with step dt:
  //   x_{k+1} approx Ad*x_k + Bd*u_k + c
  // where c is the affine offset such that evaluating the approximation
  // exactly at (x, u) reproduces integrateRk4(x, u, dt). Jacobians are
  // computed via central finite differences: the full nonlinear
  // Pacejka + trigonometric model is not worth hand-deriving analytic
  // partials for, and finite differences are far less error prone here.
  void linearizeDiscrete(
    const State & x, const Input & u, double dt,
    StateJacobian & Ad, InputJacobian & Bd, State & c) const;

  // Front/rear slip angles. vx is clamped to a floor internally to avoid
  // the 1/vx singularity near standstill; the returned angles are only
  // used for force/Jacobian evaluation, never written back into the
  // propagated state.
  void slipAngles(const State & x, double delta, double & alpha_f, double & alpha_r) const;

  // Static (no load-transfer) front/rear normal loads.
  void normalLoads(double & fz_f, double & fz_r) const;

  double lateralForceFront(double alpha_f, double fz_f) const;
  double lateralForceRear(double alpha_r, double fz_r) const;

  const VehicleParams & vehicleParams() const { return vehicle_; }

  // Solution of the steady-state cornering equations at speed vx on a path of
  // curvature kappa: the (vy, delta) pair for which the lateral dynamics are
  // actually in equilibrium with r = vx*kappa,
  //     vy_dot = (Fy_r + Fy_f*cos(delta))/m - vx*r          = 0
  //     r_dot  = (l_f*Fy_f*cos(delta) - l_r*Fy_r)/Iz        = 0
  // See steadyStateCornering().
  struct SteadyState
  {
    double vy{0.0};        // m/s, body-frame lateral velocity in the turn
    double delta{0.0};     // rad, steering angle that sustains it
    double beta{0.0};      // rad, sideslip = atan2(vy, vx)
    bool converged{true};  // false => the turn exceeds the tire model's grip
  };

  // A dynamic vehicle CANNOT corner with zero sideslip: holding curvature
  // kappa at speed vx requires a real vy and a heading offset beta from the
  // path tangent, both growing with vx^2. Feeding the MPC a reference that
  // assumes vy = 0 and psi = path tangent therefore asks for something
  // physically impossible, and the cost then trades path accuracy against
  // sideslip suppression in a fight it cannot win - the classic symptom is
  // steering chatter that appears only above some speed. This is the
  // steady-state target calculation layer of a standard tracking MPC.
  //
  // Solves the 2x2 system above for (vy, delta) by Newton iteration on the
  // full Magic Formula, seeded from the linear-tire closed form. Returns
  // converged=false (with the linear-tire seed as a best effort) when the
  // required axle forces exceed what the current tire params can produce.
  SteadyState steadyStateCornering(double vx, double kappa) const;

  // Thread-safe read of the current tire params (returns a copy - `tire_`
  // may be concurrently overwritten by setTireParams() from the
  // mpc/update_params service callback, which runs on a different
  // executor thread than the control-loop timer).
  TireParams tireParams() const;

  // Thread-safe replace of the tire params in place, called from the
  // mpc/update_params service handler once SysID/the manager have
  // validated a new identification.
  void setTireParams(const TireParams & tire);

  static constexpr double kVxFloor = 0.5;      // m/s, slip-angle/Jacobian clamp only
  static constexpr double kSlipClamp = 0.5236; // rad (~30 deg), Pacejka extrapolation guard
  // Explicit RK4 is stable for |lambda|*h < ~2.78; 1.5 keeps a margin while
  // costing at most kMaxSubsteps dynamics evaluations per integration step.
  static constexpr double kMaxLambdaDt = 1.5;
  static constexpr int kMaxSubsteps = 12;

private:
  static double pacejka(double alpha, double B, double C, double D, double E);

  VehicleParams vehicle_;
  TireParams tire_;
  mutable std::mutex tire_mutex_;
  double gravity_{9.81};
};

}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__VEHICLE_MODEL_HPP_
