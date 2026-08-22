#include "mpc_path_tracking/vehicle_model.hpp"

#include <algorithm>
#include <cmath>

namespace mpc_path_tracking
{

VehicleModel::VehicleModel(const VehicleParams & vehicle, const TireParams & tire)
: vehicle_(vehicle), tire_(tire)
{
}

VehicleModel::VehicleModel(const VehicleModel & other)
: vehicle_(other.vehicle_), tire_(other.tireParams())
{
}

VehicleModel & VehicleModel::operator=(const VehicleModel & other)
{
  if (this == &other) {
    return *this;
  }
  vehicle_ = other.vehicle_;
  setTireParams(other.tireParams());
  return *this;
}

double VehicleModel::pacejka(double alpha, double B, double C, double D, double E)
{
  const double clamped = std::clamp(alpha, -kSlipClamp, kSlipClamp);
  const double Ba = B * clamped;
  return D * std::sin(C * std::atan(Ba - E * (Ba - std::atan(Ba))));
}

void VehicleModel::slipAngles(const State & x, double delta, double & alpha_f, double & alpha_r) const
{
  const double vx = std::max(x(3), kVxFloor);
  const double vy = x(4);
  const double r = x(5);
  // Matches the sign convention used by On-Track-SysID/src/helpers/data_processing.py
  // (compute_slip_angles), which is what the identified Bf/Cf/Df/Ef/Br/Cr/Dr/Er
  // coefficients were fit against. Fy = D*sin(...) is odd in alpha, so getting
  // this sign backwards silently negates the tire force -- the restoring force
  // becomes a destabilizing one, invisible at small slip angles but diverging
  // the moment the car is meaningfully off the reference.
  alpha_f = delta - std::atan2(vy + vehicle_.l_f * r, vx);
  alpha_r = -std::atan2(vy - vehicle_.l_r * r, vx);
}

void VehicleModel::normalLoads(double & fz_f, double & fz_r) const
{
  const double wheelbase = vehicle_.l_f + vehicle_.l_r;
  fz_f = vehicle_.mass * gravity_ * vehicle_.l_r / wheelbase;
  fz_r = vehicle_.mass * gravity_ * vehicle_.l_f / wheelbase;
}

double VehicleModel::lateralForceFront(double alpha_f, double fz_f) const
{
  TireParams tire;
  {
    std::lock_guard<std::mutex> lock(tire_mutex_);
    tire = tire_;
  }
  return fz_f * pacejka(alpha_f, tire.Bf, tire.Cf, tire.Df, tire.Ef);
}

double VehicleModel::lateralForceRear(double alpha_r, double fz_r) const
{
  TireParams tire;
  {
    std::lock_guard<std::mutex> lock(tire_mutex_);
    tire = tire_;
  }
  return fz_r * pacejka(alpha_r, tire.Br, tire.Cr, tire.Dr, tire.Er);
}

TireParams VehicleModel::tireParams() const
{
  std::lock_guard<std::mutex> lock(tire_mutex_);
  return tire_;
}

void VehicleModel::setTireParams(const TireParams & tire)
{
  std::lock_guard<std::mutex> lock(tire_mutex_);
  tire_ = tire;
}

State VehicleModel::continuousDynamics(const State & x, const Input & u) const
{
  const double psi = x(2);
  const double vx = x(3);
  const double vy = x(4);
  const double r = x(5);
  const double delta = u(0);
  const double a = u(1);

  double alpha_f, alpha_r;
  slipAngles(x, delta, alpha_f, alpha_r);

  double fz_f, fz_r;
  normalLoads(fz_f, fz_r);

  const double fy_f = lateralForceFront(alpha_f, fz_f);
  const double fy_r = lateralForceRear(alpha_r, fz_r);

  State xdot;
  xdot(0) = vx * std::cos(psi) - vy * std::sin(psi);
  xdot(1) = vx * std::sin(psi) + vy * std::cos(psi);
  xdot(2) = r;
  // Longitudinal: net accel command `a` acts at the CG, plus the small
  // steering-induced front lateral-force component and the vy*r coupling.
  xdot(3) = a - (fy_f * std::sin(delta)) / vehicle_.mass + vy * r;
  xdot(4) = (fy_r + fy_f * std::cos(delta)) / vehicle_.mass - vx * r;
  xdot(5) = (vehicle_.l_f * fy_f * std::cos(delta) - vehicle_.l_r * fy_r) / vehicle_.Iz;
  return xdot;
}

int VehicleModel::integrationSubsteps(const State & x, double dt) const
{
  TireParams tire;
  {
    std::lock_guard<std::mutex> lock(tire_mutex_);
    tire = tire_;
  }
  double fz_f, fz_r;
  normalLoads(fz_f, fz_r);
  // Linear (zero-slip) cornering stiffness per axle - the steepest the
  // Magic Formula ever gets, so this over- rather than under-estimates the
  // stiffness of the lateral dynamics.
  const double c_front = fz_f * tire.Bf * tire.Cf * tire.Df;
  const double c_rear = fz_r * tire.Br * tire.Cr * tire.Dr;
  const double vx = std::max(x(3), kVxFloor);
  // |trace| of the (vy, r) subsystem's Jacobian bounds its fastest mode:
  // (Cf+Cr)/(m*vx) + (l_f^2*Cf + l_r^2*Cr)/(Iz*vx).
  const double lambda_max =
    (c_front + c_rear) / (vehicle_.mass * vx) +
    (vehicle_.l_f * vehicle_.l_f * c_front + vehicle_.l_r * vehicle_.l_r * c_rear) /
    (vehicle_.Iz * vx);
  const double steps = std::ceil(lambda_max * dt / kMaxLambdaDt);
  return std::clamp(static_cast<int>(steps), 1, kMaxSubsteps);
}

State VehicleModel::integrateRk4(const State & x, const Input & u, double dt) const
{
  // Explicit RK4 is only stable for |lambda|*h < ~2.78. A stiff tire set on a
  // small Iz puts the yaw mode at |lambda| > 100 1/s, which the control
  // period (dt = 1/control_rate_hz = 0.05 s) violates by a wide margin: the
  // "discretized" Jacobian then has a spectral radius in the tens or
  // hundreds, and over an N-stage horizon the QP the solver is handed spans
  // dozens of orders of magnitude and fails outright. Sub-stepping keeps the
  // integrator inside its stability region without shortening the horizon or
  // raising the control rate.
  const int substeps = integrationSubsteps(x, dt);
  const double h = dt / substeps;
  State xi = x;
  for (int i = 0; i < substeps; ++i) {
    const State k1 = continuousDynamics(xi, u);
    const State k2 = continuousDynamics(xi + 0.5 * h * k1, u);
    const State k3 = continuousDynamics(xi + 0.5 * h * k2, u);
    const State k4 = continuousDynamics(xi + h * k3, u);
    xi += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  }
  return xi;
}

void VehicleModel::linearizeDiscrete(
  const State & x, const Input & u, double dt,
  StateJacobian & Ad, InputJacobian & Bd, State & c) const
{
  static constexpr double kStateEps = 1e-6;
  static constexpr double kInputEps = 1e-6;

  const State x_nom = integrateRk4(x, u, dt);

  for (int i = 0; i < 6; ++i) {
    State xp = x;
    State xm = x;
    xp(i) += kStateEps;
    xm(i) -= kStateEps;
    const State fp = integrateRk4(xp, u, dt);
    const State fm = integrateRk4(xm, u, dt);
    Ad.col(i) = (fp - fm) / (2.0 * kStateEps);
  }

  for (int i = 0; i < 2; ++i) {
    Input up = u;
    Input um = u;
    up(i) += kInputEps;
    um(i) -= kInputEps;
    const State fp = integrateRk4(x, up, dt);
    const State fm = integrateRk4(x, um, dt);
    Bd.col(i) = (fp - fm) / (2.0 * kInputEps);
  }

  // Affine offset so that the affine model reproduces x_nom exactly at (x, u):
  // x_{k+1} = Ad*x_k + Bd*u_k + c, evaluated at x_k=x, u_k=u -> c = x_nom - Ad*x - Bd*u
  c = x_nom - Ad * x - Bd * u;
}

}  // namespace mpc_path_tracking
