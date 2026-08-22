"""
Non-vision friction warm-start for Pacejka cold-start.

Estimates a single scalar `mu_hat` (peak friction utilization,
mu_hat = quantile(sqrt(a_x^2 + a_y^2) / g)) over the collected on-track
data, used as a FLOOR on the static pacejka_params.yaml D default for the
node's FIRST-EVER Pacejka identification cycle's initial guess - the
non-vision analog of arXiv:2603.09399's camera-based friction prior, adapted
from arXiv:2509.15423's model-free "friction from measured acceleration
during low-slip/no-slip phases" idea.

Two accel sources (helpers/friction_warmstart.py's callers pick one via
pacejka_params.yaml's friction_warm_start.accel_source):
  - finite_diff (default): a_x/a_y computed by central-difference kinematics
    over the already-collected [vx,vy,omega,delta] buffer - no new topic or
    sensor. Works identically in SIM and on real hardware.
  - imu: a_x/a_y read directly from a real IMU's linear_acceleration (lower
    noise, matches arXiv:2509.15423's own approach) - only meaningful on
    hardware where the IMU is real. NOT used in SIM: f1tenth_simulator's own
    /imu publisher (node/simulator.cpp::pub_imu()) is an unimplemented stub
    that always publishes a zeroed message, so this path is untestable here
    and callers must fall back to finite_diff if no IMU data arrives.

D is the dimensionless peak-friction coefficient in this package's Pacejka
formula (F_y = F_z * D * sin(...), see pacejka_formula.py) - mu_hat maps
directly onto D_f_init/D_r_init, no F_z rescaling needed. Both axles get the
same mu_hat (whole-vehicle quantity, body-frame accel can't disaggregate
front/rear friction).
"""

import numpy as np

from helpers.data_processing import compute_slip_angles


def _low_slip_mask(vx, vy, omega, delta, l_f, l_r, cfg):
    alpha_f, alpha_r = compute_slip_angles(vx, vy, omega, delta, l_f, l_r)
    vx_min = cfg.get('vx_min', 1.5)
    omega_max = cfg.get('omega_max', 5.0)
    af_max = cfg.get('low_slip_alpha_f_max', 0.15)
    ar_max = cfg.get('low_slip_alpha_r_max', 0.08)
    return (
        (vx > vx_min)
        & (np.abs(omega) <= omega_max)
        & (np.abs(alpha_f) <= af_max)
        & (np.abs(alpha_r) <= ar_max)
    )


def _mu_from_accel(a_x, a_y, mask, cfg):
    """Peak utilised friction over the buffer - a LOWER BOUND on available mu.

    sqrt(a_x^2+a_y^2)/g is the friction the car actually used, not the
    friction it had available; the two coincide only at the limit. The
    original implementation took the MEDIAN of that ratio over the
    _low_slip_mask() subset - i.e. the typical utilisation of the samples
    explicitly selected for being furthest from the limit. Measured on the
    CARLA asurt_fsai driving traj_race_cl.csv under pure pursuit
    (2026-08-22): median utilisation 0.044 g at 13 m/s and 0.06 g at 21 m/s,
    against a measured axle peak of mu = 1.05. That estimator cannot return
    anything but a near-zero D no matter what the tires can do.

    So: quantile over the WHOLE buffer by default (low_slip_only: false),
    and nn_train applies the result as a floor on the static prior, never as
    a replacement that can lower it (see nn_train's warm_start_mu handling).
    Even so this stays a lower bound - on a raceline that never approaches
    the limit the peak utilisation (0.63 g measured at 21 m/s) is still well
    under the real grip, and only limit excitation can close that gap.
    """
    if bool(cfg.get('low_slip_only', False)):
        sel = mask
    else:
        sel = np.ones_like(mask, dtype=bool)
    n_used = int(np.sum(sel))
    min_samples = int(cfg.get('min_samples', 20))
    if n_used < min_samples:
        return None, n_used
    g = 9.81
    mu_samples = np.sqrt(a_x[sel] ** 2 + a_y[sel] ** 2) / g
    q = float(cfg.get('quantile', 0.99))
    return float(np.quantile(mu_samples, q)), n_used


def estimate_mu_from_buffer(data, l_f, l_r, dt, cfg):
    """Finite-difference accel_source. data: (N,4) ndarray [vx,vy,omega,delta],
    chronologically ordered (data[0]=oldest ... data[-1]=newest - the buffer
    is a genuine FIFO shift-and-append, no wraparound to correct for).
    Returns (mu_hat_or_None, n_used).
    """
    data = np.asarray(data, dtype=float)
    if data.shape[0] < 3:
        return None, 0

    vx, vy, omega, delta = data[:, 0], data[:, 1], data[:, 2], data[:, 3]

    # Central difference: lower truncation error than forward/backward on
    # already-noisy odom-derived vx/vy; the extra 1-sample lookahead latency
    # is irrelevant since this only ever runs once, offline.
    vx_dot = (vx[2:] - vx[:-2]) / (2 * dt)
    vy_dot = (vy[2:] - vy[:-2]) / (2 * dt)
    vx_c, vy_c, omega_c, delta_c = vx[1:-1], vy[1:-1], omega[1:-1], delta[1:-1]

    a_x = vx_dot - vy_c * omega_c
    a_y = vy_dot + vx_c * omega_c

    mask = _low_slip_mask(vx_c, vy_c, omega_c, delta_c, l_f, l_r, cfg)
    return _mu_from_accel(a_x, a_y, mask, cfg)


def estimate_mu_from_imu(state_samples, accel_samples, l_f, l_r, cfg):
    """IMU accel_source. state_samples: (N,4) ndarray [vx,vy,omega,delta] aligned
    1:1 with accel_samples: (N,2) ndarray [a_x,a_y] read directly from
    sensor_msgs/Imu.linear_acceleration (already body-frame - no
    differentiation, so no wraparound/central-difference concerns). Returns
    (mu_hat_or_None, n_used).
    """
    state_samples = np.asarray(state_samples, dtype=float)
    accel_samples = np.asarray(accel_samples, dtype=float)
    if state_samples.shape[0] == 0:
        return None, 0

    vx, vy, omega, delta = state_samples[:, 0], state_samples[:, 1], state_samples[:, 2], state_samples[:, 3]
    a_x, a_y = accel_samples[:, 0], accel_samples[:, 1]

    mask = _low_slip_mask(vx, vy, omega, delta, l_f, l_r, cfg)
    return _mu_from_accel(a_x, a_y, mask, cfg)
