"""Tests for the brush-model peak-friction identifier (helpers/mu_estimator.py).

Each test builds a single-track trajectory with brush tyres of a KNOWN mu,
feeds the estimator only what the car actually has - odometry states plus
body-frame accelerations, the same two streams on_track_sys_id.py buffers -
and checks that the answer comes back, or is correctly refused.

The refusal cases are the point of the module as much as the identification
is: the previous estimator (quantile of |a|/g) always returns a number, and
on a sub-limit lap that number is ~40 % low.
"""
import os
import sys

import numpy as np
import pytest

_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src')
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from helpers.mu_estimator import (  # noqa: E402
    brush_lateral_force, estimate_mu, mu_from_force_ratio)

G = 9.81
# models/SIM/SIM_pacejka.txt
MODEL = {'m': 240.0, 'I_z': 51.1, 'l_f': 0.738142, 'l_r': 0.795262, 'h_cg': 0.3}
F_ZF = MODEL['m'] * G * MODEL['l_r'] / (MODEL['l_f'] + MODEL['l_r'])
F_ZR = MODEL['m'] * G * MODEL['l_f'] / (MODEL['l_f'] + MODEL['l_r'])
# Axle cornering stiffness ~= 12.1 * F_z per rad, measured on this vehicle
# against CARLA's own per-wheel telemetry (see .wolf/STATUS.md).
C_F, C_R = 12.1 * F_ZF, 12.1 * F_ZR
DT = 0.02
CFG = {'min_samples': 200, 'utilisation_min': 0.40, 'sigma_rel_max': 0.15,
       'vx_min': 2.5, 'mu_bounds': [0.2, 2.0]}


def simulate(mu, steer_amp, duration=40.0, vx=12.0, noise=0.0, seed=0):
    """Single-track sim with brush tyres, constant vx, swept-sine steering.

    Returns (states, accels) exactly as the node buffers them: states is
    [vx, vy, omega, delta], accels is the body-frame [a_x, a_y] an IMU at the
    CG would read.
    """
    rng = np.random.default_rng(seed)
    n = int(duration / DT)
    t = np.arange(n) * DT
    # Sweep the frequency so the run visits transients as well as the
    # steady-state cornering the estimator mostly lives on.
    delta = steer_amp * np.sin(2 * np.pi * (0.1 + 0.15 * t / duration) * t)

    vy, omega = 0.0, 0.0
    states = np.zeros((n, 4))
    accels = np.zeros((n, 2))
    for k in range(n):
        d = delta[k]
        alpha_f = d - np.arctan2(vy + MODEL['l_f'] * omega, vx)
        alpha_r = -np.arctan2(vy - MODEL['l_r'] * omega, vx)
        F_yf = brush_lateral_force(alpha_f, F_ZF, C_F, mu)
        F_yr = brush_lateral_force(alpha_r, F_ZR, C_R, mu)
        a_y = (F_yf * np.cos(d) + F_yr) / MODEL['m']
        omega_dot = (MODEL['l_f'] * F_yf * np.cos(d) - MODEL['l_r'] * F_yr) / MODEL['I_z']
        states[k] = (vx, vy, omega, d)
        accels[k] = (0.0, a_y)
        vy += (a_y - vx * omega) * DT
        omega += omega_dot * DT

    if noise:
        states[:, 1] += rng.normal(0.0, noise * 0.05, n)   # vy, m/s
        states[:, 2] += rng.normal(0.0, noise * 0.01, n)   # omega, rad/s
        accels[:, 1] += rng.normal(0.0, noise * 0.10, n)   # a_y, m/s^2
    return states, accels


def test_closed_form_inverts_the_brush_model():
    """mu_from_force_ratio is the algebra the module's docstring claims."""
    mu, alpha = 1.0, 0.05
    F_y = brush_lateral_force(alpha, 1000.0, 20000.0, mu)
    mu_hat, r = mu_from_force_ratio(
        np.array([alpha]), np.array([1000.0]), np.array([F_y]), 20000.0)
    assert mu_hat[0] == pytest.approx(mu, rel=1e-6)
    assert 1.0 / 3.0 < r[0] < 1.0


@pytest.mark.parametrize('mu', [0.7, 1.05, 1.4])
def test_identifies_mu_without_reaching_the_limit(mu):
    """Steering scaled so the run peaks around 60 % grip utilisation - inside
    the 40-80 % window the literature reports as workable, and nowhere near
    the peak the utilisation estimator would need."""
    states, accels = simulate(mu, steer_amp=0.067 * mu)
    out = estimate_mu(states, MODEL, CFG, DT, accels=accels)

    assert out['ok_f'], out['front']['reason']
    assert out['ok_r'], out['rear']['reason']
    assert out['mu_f'] == pytest.approx(mu, rel=0.05)
    assert out['mu_r'] == pytest.approx(mu, rel=0.05)
    assert out['front']['C_alpha'] == pytest.approx(C_F, rel=0.10)
    assert out['rear']['C_alpha'] == pytest.approx(C_R, rel=0.10)


def test_beats_the_utilisation_bound_it_replaces():
    """The whole reason this module exists. At this amplitude the car uses
    0.47 g of the 1.05 g it has, so the old estimator reports 0.47 - 55 %
    low - while the brush fit returns mu to under 1 %."""
    mu = 1.05
    states, accels = simulate(mu, steer_amp=0.05)
    out = estimate_mu(states, MODEL, CFG, DT, accels=accels)

    assert out['mu_utilisation'] < 0.55 * mu
    assert out['ok_f']
    assert out['mu_f'] == pytest.approx(mu, rel=0.05)


def test_refuses_when_the_tyre_curve_is_still_linear():
    states, accels = simulate(1.05, steer_amp=0.02)
    out = estimate_mu(states, MODEL, CFG, DT, accels=accels)

    assert not out['ok_f'] and not out['ok_r']
    assert 'utilisation' in out['front']['reason']
    assert np.isfinite(out['mu_utilisation'])


def test_survives_sensor_noise():
    mu = 1.05
    states, accels = simulate(mu, steer_amp=0.07, noise=1.0)
    out = estimate_mu(states, MODEL, CFG, DT, accels=accels)

    assert out['ok_f'], out['front']['reason']
    assert out['mu_f'] == pytest.approx(mu, rel=0.15)


def test_falls_back_to_kinematic_accelerations_without_an_imu():
    """f1tenth_simulator's /imu is a zeroed stub, so an all-zero accel buffer
    has to degrade to differentiated odometry rather than to garbage.

    The looser tolerance is the cost of that fallback: the Savitzky-Golay
    derivative of v_y smooths the peaks of a_y, which biases mu high by
    ~10 %. Prefer a real IMU where one exists.
    """
    mu = 1.05
    states, _ = simulate(mu, steer_amp=0.07)
    out = estimate_mu(states, MODEL, CFG, DT, accels=np.zeros((states.shape[0], 2)))

    assert out['accel_source'] == 'kinematic'
    assert out['ok_f'], out['front']['reason']
    assert out['mu_f'] == pytest.approx(mu, rel=0.15)
