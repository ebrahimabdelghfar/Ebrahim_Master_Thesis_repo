"""
Multi-step predictor for dynamic single-track vehicle model.

Implements the prediction methodology from paper 2411.17508v1:
    - States:  x = [v_y, omega]
    - Inputs:  u = [v_x, delta]
    - Pacejka Magic Formula lateral tire forces
    - Euler integration for discrete-time state transition (Eq. 4)

One-step prediction (steps=1) is identical to the existing
on_track_sys_id.py publish_estimates() logic.

Multi-step prediction (steps>1) performs open-loop Euler roll-forward
using model predictions as inputs to subsequent steps (no sensor feedback).
"""

import numpy as np


def pacejka_formula(params, alpha, F_z):
    """Pacejka Magic Formula (Eq. 2 in paper).

    Parameters
    ----------
    params : list
        [B, C, D, E] Pacejka coefficients.
    alpha : float
        Slip angle [rad].
    F_z : float
        Normal force [N].

    Returns
    -------
    float
        Lateral tire force [N].
    """
    B, C, D, E = params[0], params[1], params[2], params[3]
    return F_z * D * np.sin(
        C * np.arctan(B * alpha - E * (B * alpha - np.arctan(B * alpha)))
    )


def predict_one_step(v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params, dt):
    """Predict next (v_y, omega) using one Euler step (Eq. 4 in paper).

    Parameters
    ----------
    v_x : float
        Longitudinal velocity [m/s].
    v_y : float
        Lateral velocity [m/s].
    omega : float
        Yaw rate [rad/s].
    delta : float
        Steering angle [rad].
    C_Pf : list
        Front Pacejka params [B, C, D, E].
    C_Pr : list
        Rear Pacejka params [B, C, D, E].
    vehicle_params : dict
        Must contain keys: mass, I_z, l_f, l_r, l_wb.
    dt : float
        Sampling time [s].

    Returns
    -------
    tuple
        (v_y_next, omega_next) predicted state.
    """
    m = vehicle_params['mass']
    I_z = vehicle_params['I_z']
    l_f = vehicle_params['l_f']
    l_r = vehicle_params['l_r']
    l_wb = vehicle_params['l_wb']

    # Normal forces from static weight distribution
    F_zf = m * 9.81 * l_r / l_wb
    F_zr = m * 9.81 * l_f / l_wb

    # Slip angles (Eq. 3)
    alpha_f = -np.arctan((v_y + omega * l_f) / v_x) + delta
    alpha_r = -np.arctan((v_y - omega * l_r) / v_x)

    # Pacejka lateral forces (Eq. 2)
    F_f = pacejka_formula(C_Pf, alpha_f, F_zf)
    F_r = pacejka_formula(C_Pr, alpha_r, F_zr)

    # Dynamics derivatives (Eq. 1)
    v_y_dot = (1.0 / m) * (F_r + F_f * np.cos(delta) - m * v_x * omega)
    omega_dot = (1.0 / I_z) * (F_f * l_f * np.cos(delta) - F_r * l_r)

    # Euler integration (Eq. 4)
    v_y_next = v_y + v_y_dot * dt
    omega_next = omega + omega_dot * dt

    return v_y_next, omega_next


def predict_multi_step(v_x, v_y_init, omega_init, delta, C_Pf, C_Pr,
                       vehicle_params, dt, steps):
    """Predict (v_y, omega) over multiple Euler steps (open-loop).

    For multi-step prediction, the model uses its own predictions as
    state inputs for subsequent steps (open-loop roll-forward).
    v_x and delta are held constant (from the initial measurement)
    since future inputs are not available.

    Parameters
    ----------
    v_x : float
        Longitudinal velocity [m/s] (held constant across steps).
    v_y_init : float
        Initial lateral velocity [m/s].
    omega_init : float
        Initial yaw rate [rad/s].
    delta : float
        Steering angle [rad] (held constant across steps).
    C_Pf : list
        Front Pacejka params [B, C, D, E].
    C_Pr : list
        Rear Pacejka params [B, C, D, E].
    vehicle_params : dict
        Must contain keys: mass, I_z, l_f, l_r, l_wb.
    dt : float
        Sampling time [s].
    steps : int
        Number of prediction steps (1 = one-step prediction).

    Returns
    -------
    tuple
        (v_y_pred, omega_pred) at the final step.
    """
    v_y = v_y_init
    omega = omega_init

    for _ in range(steps):
        v_y, omega = predict_one_step(
            v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params, dt
        )

    return v_y, omega
