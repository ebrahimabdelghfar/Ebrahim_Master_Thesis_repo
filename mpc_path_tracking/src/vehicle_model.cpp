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

VehicleModel::SteadyState VehicleModel::steadyStateCornering(double vx, double kappa) const
{
  SteadyState ss;

  const double v = std::max(vx, kVxFloor);
  const double r = v * kappa;
  if (std::abs(r) < 1e-9) {
    return ss;   // straight line: vy = delta = beta = 0 is exact
  }

  TireParams tire;
  {
    std::lock_guard<std::mutex> lock(tire_mutex_);
    tire = tire_;
  }
  double fz_f, fz_r;
  normalLoads(fz_f, fz_r);
  const double l_f = vehicle_.l_f;
  const double l_r = vehicle_.l_r;
  const double wheelbase = l_f + l_r;

  // --- seed from the linear-tire closed form -------------------------------
  // Force balance m*a_y = Fy_f + Fy_r with moment balance l_f*Fy_f = l_r*Fy_r
  // splits the demand across the axles by the static load ratio.
  const double a_y = v * r;
  const double fy_f_req = vehicle_.mass * a_y * l_r / wheelbase;
  const double fy_r_req = vehicle_.mass * a_y * l_f / wheelbase;
  // d/dalpha of the Magic Formula at alpha = 0 is Fz*B*C*D.
  const double c_front = fz_f * tire.Bf * tire.Cf * tire.Df;
  const double c_rear = fz_r * tire.Br * tire.Cr * tire.Dr;
  const double alpha_f_seed = fy_f_req / std::max(c_front, 1e-6);
  const double alpha_r_seed = fy_r_req / std::max(c_rear, 1e-6);
  ss.vy = l_r * r - v * std::tan(std::clamp(alpha_r_seed, -kSlipClamp, kSlipClamp));
  ss.delta = alpha_f_seed + std::atan2(ss.vy + l_f * r, v);
  ss.beta = std::atan2(ss.vy, v);

  // The turn is simply not achievable if it needs more grip than the axles
  // can make - report it rather than let Newton wander over the saturated
  // (and non-monotonic) part of the Magic Formula.
  const double fy_f_cap = fz_f * std::abs(tire.Df);
  const double fy_r_cap = fz_r * std::abs(tire.Dr);
  if (std::abs(fy_f_req) > fy_f_cap || std::abs(fy_r_req) > fy_r_cap) {
    ss.converged = false;
    return ss;
  }

  // --- Newton on the full nonlinear residual -------------------------------
  // F1 = vy_dot, F2 = r_dot, both must vanish at a true equilibrium.
  const auto residual = [&](double vy, double delta, double & f1, double & f2) {
      const double alpha_f = delta - std::atan2(vy + l_f * r, v);
      const double alpha_r = -std::atan2(vy - l_r * r, v);
      const double fy_f = fz_f * pacejka(alpha_f, tire.Bf, tire.Cf, tire.Df, tire.Ef);
      const double fy_r = fz_r * pacejka(alpha_r, tire.Br, tire.Cr, tire.Dr, tire.Er);
      const double cd = std::cos(delta);
      f1 = (fy_r + fy_f * cd) / vehicle_.mass - v * r;
      f2 = (l_f * fy_f * cd - l_r * fy_r) / vehicle_.Iz;
    };

  // Residuals are accelerations; scale the tolerance to the demand so the
  // same absolute number is not absurdly tight at 5 m/s and loose at 30.
  const double tol = 1e-9 * std::max(1.0, std::abs(a_y));
  constexpr int kMaxIters = 30;
  constexpr double kFdEps = 1e-7;

  double vy = ss.vy;
  double delta = ss.delta;
  bool converged = false;
  for (int it = 0; it < kMaxIters; ++it) {
    double f1, f2;
    residual(vy, delta, f1, f2);
    if (std::abs(f1) < tol && std::abs(f2) < tol) {
      converged = true;
      break;
    }
    double f1_v, f2_v, f1_d, f2_d;
    residual(vy + kFdEps, delta, f1_v, f2_v);
    residual(vy, delta + kFdEps, f1_d, f2_d);
    const double j11 = (f1_v - f1) / kFdEps;
    const double j12 = (f1_d - f1) / kFdEps;
    const double j21 = (f2_v - f2) / kFdEps;
    const double j22 = (f2_d - f2) / kFdEps;
    const double det = j11 * j22 - j12 * j21;
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
      break;   // singular - keep the best iterate, flagged below
    }
    const double d_vy = (-f1 * j22 + f2 * j12) / det;
    const double d_delta = (-f2 * j11 + f1 * j21) / det;
    // Damping keeps a bad Jacobian from throwing the iterate into the
    // saturated region, from which the Magic Formula cannot recover.
    const double step = std::min(
      1.0, 0.5 / std::max({1e-9, std::abs(d_vy) / std::max(1.0, std::abs(v)),
        std::abs(d_delta) / 0.2}));
    vy += step * d_vy;
    delta += step * d_delta;
    if (!std::isfinite(vy) || !std::isfinite(delta)) {
      break;
    }
  }

  if (converged) {
    ss.vy = vy;
    ss.delta = delta;
    ss.beta = std::atan2(vy, v);
  }
  ss.converged = converged;
  return ss;
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
