"""
Peak-friction (mu) identification from IMU + odometry only.

Replaces the friction-utilisation heuristic in friction_warmstart.py, which
reports quantile(|a|/g) - the friction the car *used*, a lower bound that
only reaches the available friction if the car actually reaches the limit.
On this vehicle a full raceline lap under MPC peaks at 0.63 g against a
measured axle peak of 1.05, so that estimator is short by ~40 % by
construction.

Method (slip-based lateral-dynamics friction estimation, the dominant class
in Acosta et al., "Road Friction Virtual Sensing", Appl. Sci. 7(12):1230,
2017, Sec. 3.2):

  1. Axle lateral forces from Newton-Euler on the single-track body, using
     only IMU lateral acceleration and yaw acceleration - no tyre model, no
     force sensor:
         F_yf = (I_z*r_dot + l_r*m*a_y) / (l_wb*cos(delta))
         F_yr = (l_f*m*a_y - I_z*r_dot) / l_wb
     Axle vertical loads carry the longitudinal transfer m*a_x*h_cg/l_wb.
  2. Slip angles from the odometry states already in the buffer.
  3. Per axle, a joint nonlinear least-squares fit of the Fiala brush model
         F_y = mu*F_z * (1 - (1 - xi)^3) * sgn(alpha),
         xi   = C_alpha*|tan alpha| / (3*mu*F_z),   clipped to 1
     for BOTH the cornering stiffness C_alpha and the peak friction mu.

The reason this can beat the utilisation bound is that mu enters the brush
model through the *curvature* of F_y(alpha), not only through its asymptote:
the rate at which the axle force falls below the linear extrapolation
C_alpha*tan(alpha) identifies the peak without the car ever reaching it.
Concretely, writing r = |F_y| / (C_alpha*|tan alpha|), the model inverts in
closed form to

    mu = 2k / (3 - sqrt(3*(4r - 1))),      k = C_alpha*|tan alpha| / (3*F_z)

so a 30 % departure from linearity (r = 0.7) already pins mu. That closed
form is only used here for the excitation diagnostics - fitting C_alpha and
mu jointly avoids the bias a two-stage fit would inherit from estimating
C_alpha on data that is itself already slightly nonlinear.

Observability limits, and why the gate exists. The same review reports that
lateral-dynamics estimators need real excitation: Ray (via Rajamani et al.)
fails below |a_y| < 0.4 g "due to the proximity of the tyre force curves in
the low slip regions"; Han et al. need normalised lateral force 0.30-0.40;
Choi et al. track mu-jumps at a sustained 4 m/s^2; Acosta and Kanarachos
gate at |a_y| > 1.5 m/s^2 and report 40-80 % grip utilisation as the
working range. The closed form above says the same thing quantitatively -
d(mu)/dr grows without bound as r -> 1, and the default r_max of 0.85 is
exactly the ~40 % grip utilisation those papers converge on. So this module
returns an answer only when the manoeuvre supports one, and reports the
utilisation floor otherwise rather than inventing a number.

Assumptions, all of which bias mu if violated:
  - Flat road. A bank angle leaks g*sin(phi) into the IMU's a_y.
  - The IMU sits at (or near) the CG; a longitudinal offset x_imu adds
    r_dot*x_imu to the measured a_y.
  - Pure lateral slip. Combined slip shrinks the lateral capacity, so
    samples with |a_x| above ax_max_g are dropped rather than corrected -
    the axle longitudinal force split needed for a friction-ellipse
    correction is not observable from this sensor set.
"""

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from helpers.data_processing import compute_slip_angles

G = 9.81


def brush_lateral_force(alpha, F_z, C_alpha, mu):
    """Fiala brush model lateral force. Saturates at mu*F_z."""
    z = np.tan(alpha)
    xi = np.clip(C_alpha * np.abs(z) / (3.0 * mu * np.maximum(F_z, 1e-6)), 0.0, 1.0)
    return mu * F_z * (1.0 - (1.0 - xi) ** 3) * np.sign(z)


def axle_lateral_forces(a_y, yaw_accel, delta, m, I_z, l_f, l_r):
    """Newton-Euler axle forces. F_yf is in the front WHEEL frame."""
    l_wb = l_f + l_r
    F_yf = (I_z * yaw_accel + l_r * m * a_y) / (l_wb * np.maximum(np.cos(delta), 0.1))
    F_yr = (l_f * m * a_y - I_z * yaw_accel) / l_wb
    return F_yf, F_yr


def axle_vertical_loads(a_x, m, l_f, l_r, h_cg):
    """Static split plus longitudinal load transfer."""
    l_wb = l_f + l_r
    F_zf = m * (G * l_r - a_x * h_cg) / l_wb
    F_zr = m * (G * l_f + a_x * h_cg) / l_wb
    return np.maximum(F_zf, 1.0), np.maximum(F_zr, 1.0)


def mu_from_force_ratio(alpha, F_z, F_y, C_alpha):
    """Closed-form brush inversion, per sample. Diagnostic only.

    Returns (mu, r) with mu = nan where the sample carries no information
    (r outside the model's own [1/3, 1] range).
    """
    lin = C_alpha * np.abs(np.tan(alpha))
    with np.errstate(divide='ignore', invalid='ignore'):
        r = np.abs(F_y) / lin
        k = lin / (3.0 * F_z)
        mu = 2.0 * k / (3.0 - np.sqrt(np.maximum(3.0 * (4.0 * r - 1.0), 0.0)))
    # Below 1/3 the brush model is fully sliding: the force IS mu*F_z.
    mu = np.where(r < 1.0 / 3.0, np.abs(F_y) / F_z, mu)
    return mu, r


def _yaw_accel(omega, dt, cfg):
    """Savitzky-Golay derivative of the yaw rate.

    A plain difference of odometry yaw rate puts noise straight into
    I_z*r_dot, which is a ~10 % term in F_yf on this car - so the derivative
    is taken by local polynomial fit rather than by differencing.
    """
    n = omega.size
    window = int(cfg.get('savgol_window', 21))
    window = min(window if window % 2 else window + 1, n if n % 2 else n - 1)
    poly = int(cfg.get('savgol_poly', 2))
    if window < poly + 2:
        return np.gradient(omega, dt)
    return savgol_filter(omega, window, poly, deriv=1, delta=dt)


def _body_accel_from_states(vx, vy, omega, dt, cfg):
    """Kinematic a_x/a_y, used when no IMU is available."""
    window = int(cfg.get('savgol_window', 21))
    n = vx.size
    window = min(window if window % 2 else window + 1, n if n % 2 else n - 1)
    poly = int(cfg.get('savgol_poly', 2))
    if window < poly + 2:
        vx_dot, vy_dot = np.gradient(vx, dt), np.gradient(vy, dt)
    else:
        vx_dot = savgol_filter(vx, window, poly, deriv=1, delta=dt)
        vy_dot = savgol_filter(vy, window, poly, deriv=1, delta=dt)
    return vx_dot - vy * omega, vy_dot + vx * omega


def _fit_axle(alpha, F_z, F_y, cfg, mu_floor):
    """Joint NLLS of (C_alpha, mu) on the brush curve for one axle."""
    n = int(alpha.size)
    out = {'ok': False, 'n': n, 'mu': float('nan'), 'C_alpha': float('nan'),
           'sigma_mu': float('inf'), 'utilisation': 0.0, 'reason': ''}
    if n < int(cfg.get('min_samples', 200)):
        out['reason'] = f"{n} usable samples < min_samples={cfg.get('min_samples', 200)}"
        return out

    C_lo, C_hi = [float(v) for v in cfg.get('C_bounds', [2.0e3, 3.0e5])]
    mu_lo, mu_hi = [float(v) for v in cfg.get('mu_bounds', [0.2, 2.0])]

    z = np.tan(alpha)
    lin = np.abs(alpha) <= float(cfg.get('alpha_linear_max', 0.02))
    sel = lin if np.count_nonzero(lin) >= 10 else np.ones(n, dtype=bool)
    C0 = float(np.clip(np.sum(z[sel] * F_y[sel]) / max(np.sum(z[sel] ** 2), 1e-9), C_lo, C_hi))
    mu0 = float(np.clip(max(mu_floor, np.max(np.abs(F_y / F_z))), mu_lo, mu_hi))

    def residual(p):
        return (brush_lateral_force(alpha, F_z, p[0], p[1]) - F_y) / F_z

    sol = least_squares(
        residual, x0=[C0, mu0], bounds=([C_lo, mu_lo], [C_hi, mu_hi]),
        loss='soft_l1', f_scale=float(cfg.get('f_scale', 0.05)), x_scale=[C0, 1.0])
    C_hat, mu_hat = float(sol.x[0]), float(sol.x[1])

    # Parameter covariance from the Gauss-Newton approximation. sigma_mu is
    # the whole point of the gate: at low excitation the fit still returns a
    # number, and this is what says it means nothing.
    dof = max(n - 2, 1)
    s2 = 2.0 * float(sol.cost) / dof
    try:
        sigma = np.sqrt(np.abs(np.diag(np.linalg.inv(sol.jac.T @ sol.jac) * s2)))
    except np.linalg.LinAlgError:
        sigma = np.array([np.inf, np.inf])

    utilisation = float(np.max(np.abs(F_y) / (mu_hat * F_z)))
    span = max(mu_hi - mu_lo, 1e-9)
    railed = min(mu_hat - mu_lo, mu_hi - mu_hat) / span < 1e-3

    out.update({'mu': mu_hat, 'C_alpha': C_hat, 'sigma_mu': float(sigma[1]),
                'sigma_C': float(sigma[0]), 'utilisation': utilisation,
                'rmse': float(np.sqrt(np.mean(residual(sol.x) ** 2)))})

    util_min = float(cfg.get('utilisation_min', 0.40))
    sigma_max = float(cfg.get('sigma_rel_max', 0.15))
    if railed:
        out['reason'] = f"mu={mu_hat:.3f} railed on mu_bounds - not identified by the data"
    elif utilisation < util_min:
        out['reason'] = (f"peak grip utilisation {utilisation:.2f} < {util_min:.2f} - "
                         "the tyre curve is still linear here, mu is not observable")
    elif sigma[1] / max(mu_hat, 1e-6) > sigma_max:
        out['reason'] = (f"sigma_mu/mu = {sigma[1] / mu_hat:.2f} > {sigma_max:.2f} - "
                         "fit is not confident")
    else:
        out['ok'] = True
    return out


def estimate_mu(states, model, cfg, dt, accels=None):
    """Identify per-axle peak friction from the on-track buffer.

    states: (N,4) [vx, vy, omega, delta], chronological.
    accels: (N,2) [a_x, a_y] body-frame IMU accelerations, index-aligned with
            `states`. None (or all-zero) falls back to kinematic accelerations
            differentiated from the states.
    Returns a dict; `mu_f`/`mu_r` are NaN unless the matching `ok_f`/`ok_r`
    is True. `mu_utilisation` is always present and is a hard lower bound.
    """
    states = np.asarray(states, dtype=float)
    n = states.shape[0]
    result = {'mu_f': float('nan'), 'mu_r': float('nan'), 'ok_f': False, 'ok_r': False,
              'mu_utilisation': float('nan'), 'accel_source': 'kinematic', 'front': {},
              'rear': {}, 'imu_vs_kinematic_ay_rms': float('nan')}
    if n < 3:
        result['reason'] = f"buffer holds {n} samples"
        return result

    vx, vy, omega, delta = states[:, 0], states[:, 1], states[:, 2], states[:, 3]
    m, I_z = float(model['m']), float(model['I_z'])
    l_f, l_r = float(model['l_f']), float(model['l_r'])
    h_cg = float(model.get('h_cg', 0.0)) if cfg.get('use_load_transfer', True) else 0.0

    a_x_kin, a_y_kin = _body_accel_from_states(vx, vy, omega, dt, cfg)
    if accels is not None and np.any(np.asarray(accels, dtype=float)):
        accels = np.asarray(accels, dtype=float)
        a_x, a_y = accels[:, 0], accels[:, 1]
        result['accel_source'] = 'imu'
        # Cross-check, logged by the caller: a persistent offset here is an
        # IMU bias or a mounting offset, both of which bias mu directly.
        result['imu_vs_kinematic_ay_rms'] = float(np.sqrt(np.mean((a_y - a_y_kin) ** 2)))
    else:
        a_x, a_y = a_x_kin, a_y_kin

    result['mu_utilisation'] = float(np.quantile(
        np.sqrt(a_x ** 2 + a_y ** 2) / G, float(cfg.get('utilisation_quantile', 0.99))))

    r_dot = _yaw_accel(omega, dt, cfg)
    alpha_f, alpha_r = compute_slip_angles(vx, vy, omega, delta, l_f, l_r)
    F_yf, F_yr = axle_lateral_forces(a_y, r_dot, delta, m, I_z, l_f, l_r)
    F_zf, F_zr = axle_vertical_loads(a_x, m, l_f, l_r, h_cg)

    alpha_min = float(cfg.get('alpha_min', 0.005))
    base = (
        (vx > float(cfg.get('vx_min', 2.5)))
        & (np.abs(a_x) <= float(cfg.get('ax_max_g', 0.35)) * G)
        & np.isfinite(a_y) & np.isfinite(r_dot)
    )
    # A force opposing its own slip angle is a sign error or pure noise, and
    # the brush model cannot represent it.
    m_f = base & (np.abs(alpha_f) >= alpha_min) & (np.sign(alpha_f) == np.sign(F_yf))
    m_r = base & (np.abs(alpha_r) >= alpha_min) & (np.sign(alpha_r) == np.sign(F_yr))

    floor = result['mu_utilisation']
    result['front'] = _fit_axle(alpha_f[m_f], F_zf[m_f], F_yf[m_f], cfg, floor)
    result['rear'] = _fit_axle(alpha_r[m_r], F_zr[m_r], F_yr[m_r], cfg, floor)
    for side, key in (('front', 'f'), ('rear', 'r')):
        result[f'ok_{key}'] = bool(result[side]['ok'])
        result[f'mu_{key}'] = float(result[side]['mu'])
    return result
