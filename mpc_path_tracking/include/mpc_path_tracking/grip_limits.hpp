#ifndef MPC_PATH_TRACKING__GRIP_LIMITS_HPP_
#define MPC_PATH_TRACKING__GRIP_LIMITS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include "mpc_path_tracking/vehicle_model.hpp"

namespace mpc_path_tracking
{
namespace grip
{

constexpr double kGravity = 9.81;

/** Writes the static (no load-transfer) front and rear axle normal loads. */
inline void normalLoads(const VehicleParams & vp, double & fz_f, double & fz_r)
{
  const double wheelbase = vp.l_f + vp.l_r;
  fz_f = vp.mass * kGravity * vp.l_r / wheelbase;
  fz_r = vp.mass * kGravity * vp.l_f / wheelbase;
}

/** Peak lateral acceleration `tire` can carry with both axles at their Magic-Formula peak. */
inline double gripCeiling(const TireParams & tire, const VehicleParams & vp)
{
  double fz_f = 0.0, fz_r = 0.0;
  normalLoads(vp, fz_f, fz_r);
  return (std::abs(tire.Df) * fz_f + std::abs(tire.Dr) * fz_r) / vp.mass;
}

/**
 * Peak lateral acceleration the binding axle can carry while accelerating at `a_x`.
 * Steady-state moment balance pins each axle's demand to `F_y/F_z = a_y/g`, so one axle's
 * surplus can never cover the other's deficit and `gripCeiling`'s sum is reachable only at
 * `Df == Dr` with no load transfer. Braking moves `m*a_x*h_cg/L` off the rear, which is the
 * margin a trail-braked apex spends and the sum still reports as available.
 */
inline double axleGripCeiling(const TireParams & tire, const VehicleParams & vp, double a_x)
{
  const double transfer = a_x * vp.h_cg;
  const double front = std::abs(tire.Df) * (kGravity * vp.l_r - transfer) / vp.l_r;
  const double rear = std::abs(tire.Dr) * (kGravity * vp.l_f + transfer) / vp.l_f;
  return std::max(0.0, std::min(front, rear));
}

/**
 * Oversteer critical speed of the linear single-track model, or infinity when the axle
 * balance is understeering. Above it every horizon stage linearized there diverges.
 */
inline double criticalSpeed(const TireParams & tire, const VehicleParams & vp)
{
  double fz_f = 0.0, fz_r = 0.0;
  normalLoads(vp, fz_f, fz_r);
  // d/dalpha of the Magic Formula at alpha = 0 is Fz*B*C*D, the axle's cornering stiffness
  const double c_front = fz_f * tire.Bf * tire.Cf * tire.Df;
  const double c_rear = fz_r * tire.Br * tire.Cr * tire.Dr;
  const double margin = vp.l_f * c_front - vp.l_r * c_rear;
  if (margin <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return (vp.l_f + vp.l_r) * std::sqrt(c_front * c_rear / (vp.mass * margin));
}

/**
 * Slip angle at which one axle's Magic Formula peaks, found by scanning [0, kSlipClamp].
 * The closed form only holds for E = 0, and E > 1 makes the implicit equation
 * non-monotone, so the scan is what keeps every identified coefficient set admissible.
 */
inline double peakSlipAngle(double B, double C, double D, double E)
{
  constexpr int kSamples = 1024;
  double best_alpha = VehicleModel::kSlipClamp;
  double best_force = -std::numeric_limits<double>::infinity();
  for (int i = 1; i <= kSamples; ++i) {
    const double alpha = VehicleModel::kSlipClamp * static_cast<double>(i) / kSamples;
    const double Ba = B * alpha;
    const double force = D * std::sin(C * std::atan(Ba - E * (Ba - std::atan(Ba))));
    if (force > best_force) {
      best_force = force;
      best_alpha = alpha;
    }
  }
  return best_alpha;
}

/**
 * Derates `base_util` by how much further out the identified tire's peak slip angle sits
 * than the reference tire's. An axle only reaches D at its peak slip, and a 20 Hz loop
 * behind the drivetrain lag cannot chase a peak that has moved, so the reachable fraction
 * of the ceiling falls with the peak's position. Unity when the shape is unchanged.
 */
inline double shapeUtilization(
  const TireParams & tire, const TireParams & ref_tire, double base_util)
{
  const double ap_f = peakSlipAngle(tire.Bf, tire.Cf, tire.Df, tire.Ef);
  const double ap_r = peakSlipAngle(tire.Br, tire.Cr, tire.Dr, tire.Er);
  const double ref_f = peakSlipAngle(ref_tire.Bf, ref_tire.Cf, ref_tire.Df, ref_tire.Ef);
  const double ref_r = peakSlipAngle(ref_tire.Br, ref_tire.Cr, ref_tire.Dr, ref_tire.Er);
  if (!(ap_f > 0.0) || !(ap_r > 0.0)) {
    return base_util;
  }
  const double shape = std::min(std::min(1.0, ref_f / ap_f), std::min(1.0, ref_r / ap_r));
  return base_util * shape;
}

/**
 * Shrinks the utilization in proportion to the relative uncertainty of the friction fit,
 * never below `util_min` and never above the untightened value. Returns `util` unchanged
 * when no usable estimate is available.
 */
inline double applySigmaTightening(
  double util, double mu, double sigma_mu, double gain, double util_min)
{
  if (!(mu > 0.0) || !std::isfinite(sigma_mu) || sigma_mu < 0.0) {
    return util;
  }
  return std::clamp(util * (1.0 - gain * sigma_mu / mu), std::min(util_min, util), util);
}

/**
 * Applies a new grip ceiling asymmetrically: a drop takes effect at once, a recovery ramps
 * at `rise_rate` per second. Grip that is gone is gone now, and grip that has come back is
 * a claim the next identification has yet to confirm.
 */
inline double rateLimitedCeiling(double prev, double target, double rise_rate, double dt)
{
  if (!(prev > 0.0) || target <= prev || !(rise_rate > 0.0) || !(dt > 0.0)) {
    return target;
  }
  return std::min(target, prev + rise_rate * dt);
}

}  // namespace grip
}  // namespace mpc_path_tracking

#endif  // MPC_PATH_TRACKING__GRIP_LIMITS_HPP_
