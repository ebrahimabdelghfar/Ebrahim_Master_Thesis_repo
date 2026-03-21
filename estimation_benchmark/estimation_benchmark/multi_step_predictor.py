"""
Multi-step predictor for dynamic single-track vehicle model.

Implements the prediction methodology from paper 2411.17508v1:
    - States:  x = [v_y, omega]
    - Inputs:  u = [v_x, delta]
    - Pacejka Magic Formula lateral tire forces
    - Configurable integration: Euler (1st-order), Heun/RK2 (2nd-order), RK4 (4th-order)

One-step prediction (steps=1) is identical to the existing
on_track_sys_id.py publish_estimates() logic (when using Euler).

Multi-step prediction (steps>1) performs open-loop roll-forward
using model predictions as inputs to subsequent steps (no sensor feedback).

Supported integration methods:
    - "euler" : Forward Euler (1st-order, paper Eq. 4)
        x_{k+1} = x_k + dt * f(x_k)
        Error: O(dt^2) per step, O(dt) global

    - "heun"  : Heun's method / Improved Euler (2nd-order)
        k1 = f(x_k)
        k2 = f(x_k + dt * k1)
        x_{k+1} = x_k + dt/2 * (k1 + k2)
        Error: O(dt^3) per step, O(dt^2) global

    - "rk4"   : Classical Runge-Kutta (4th-order)
        k1 = f(x_k)
        k2 = f(x_k + dt/2 * k1)
        k3 = f(x_k + dt/2 * k2)
        k4 = f(x_k + dt * k3)
        x_{k+1} = x_k + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        Error: O(dt^5) per step, O(dt^4) global
"""

import numpy as np

# Valid integration method names
VALID_METHODS = ('euler', 'heun', 'rk4')


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


def _dynamics(v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params):
    """Compute the state derivatives (v_y_dot, omega_dot) from Eq. 1.

    This is the continuous-time dynamics f(x, u) that all integration
    methods evaluate. Factored out so Euler, Heun, and RK4 share the
    same physics with zero duplication.

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

    Returns
    -------
    tuple
        (v_y_dot, omega_dot) continuous-time derivatives.
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

    return v_y_dot, omega_dot


# ──────────────────────────────────────────────────────────────────────
# Integration methods
# ──────────────────────────────────────────────────────────────────────

def _step_euler(v_x, v_y, omega, delta, C_Pf, C_Pr, vp, dt):
    """Forward Euler (1st-order).

    x_{k+1} = x_k + dt * f(x_k)
    """
    vy_dot, om_dot = _dynamics(v_x, v_y, omega, delta, C_Pf, C_Pr, vp)
    return v_y + vy_dot * dt, omega + om_dot * dt


def _step_heun(v_x, v_y, omega, delta, C_Pf, C_Pr, vp, dt):
    """Heun's method / Improved Euler (2nd-order).

    k1 = f(x_k)
    k2 = f(x_k + dt * k1)
    x_{k+1} = x_k + (dt / 2) * (k1 + k2)
    """
    # Stage 1
    k1_vy, k1_om = _dynamics(v_x, v_y, omega, delta, C_Pf, C_Pr, vp)

    # Stage 2: evaluate at the Euler-predicted endpoint
    v_y_tilde = v_y + dt * k1_vy
    omega_tilde = omega + dt * k1_om
    k2_vy, k2_om = _dynamics(v_x, v_y_tilde, omega_tilde, delta, C_Pf, C_Pr, vp)

    # Weighted average
    return (
        v_y + 0.5 * dt * (k1_vy + k2_vy),
        omega + 0.5 * dt * (k1_om + k2_om),
    )


def _step_rk4(v_x, v_y, omega, delta, C_Pf, C_Pr, vp, dt):
    """Classical 4th-order Runge-Kutta.

    k1 = f(x_k)
    k2 = f(x_k + dt/2 * k1)
    k3 = f(x_k + dt/2 * k2)
    k4 = f(x_k + dt * k3)
    x_{k+1} = x_k + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4)
    """
    h2 = dt * 0.5

    k1_vy, k1_om = _dynamics(v_x, v_y, omega, delta, C_Pf, C_Pr, vp)
    k2_vy, k2_om = _dynamics(v_x, v_y + h2 * k1_vy, omega + h2 * k1_om,
                              delta, C_Pf, C_Pr, vp)
    k3_vy, k3_om = _dynamics(v_x, v_y + h2 * k2_vy, omega + h2 * k2_om,
                              delta, C_Pf, C_Pr, vp)
    k4_vy, k4_om = _dynamics(v_x, v_y + dt * k3_vy, omega + dt * k3_om,
                              delta, C_Pf, C_Pr, vp)

    return (
        v_y + (dt / 6.0) * (k1_vy + 2.0 * k2_vy + 2.0 * k3_vy + k4_vy),
        omega + (dt / 6.0) * (k1_om + 2.0 * k2_om + 2.0 * k3_om + k4_om),
    )


# Dispatch table
_INTEGRATORS = {
    'euler': _step_euler,
    'heun': _step_heun,
    'rk4': _step_rk4,
}


def get_integrator(method: str):
    """Return the integrator function for the given method name.

    Parameters
    ----------
    method : str
        One of 'euler', 'heun', or 'rk4'.

    Returns
    -------
    callable
        Step function with signature (v_x, v_y, omega, delta, C_Pf, C_Pr, vp, dt).

    Raises
    ------
    ValueError
        If *method* is not a recognised integration method.
    """
    key = method.strip().lower()
    if key not in _INTEGRATORS:
        raise ValueError(
            f"Unknown integration method '{method}'. "
            f"Supported: {', '.join(VALID_METHODS)}"
        )
    return _INTEGRATORS[key]


# ──────────────────────────────────────────────────────────────────────
# Public API (backward-compatible)
# ──────────────────────────────────────────────────────────────────────

def predict_one_step(v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params, dt,
                     method='euler'):
    """Predict next (v_y, omega) using one integration step.

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
    method : str, optional
        Integration method: 'euler', 'heun', or 'rk4' (default: 'euler').

    Returns
    -------
    tuple
        (v_y_next, omega_next) predicted state.
    """
    step_fn = get_integrator(method)
    return step_fn(v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params, dt)


def predict_multi_step(v_x, v_y_init, omega_init, delta, C_Pf, C_Pr,
                       vehicle_params, dt, steps, method='euler'):
    """Predict (v_y, omega) over multiple integration steps (open-loop).

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
    method : str, optional
        Integration method: 'euler', 'heun', or 'rk4' (default: 'euler').

    Returns
    -------
    tuple
        (v_y_pred, omega_pred) at the final step.
    """
    step_fn = get_integrator(method)
    v_y = v_y_init
    omega = omega_init

    for _ in range(steps):
        v_y, omega = step_fn(
            v_x, v_y, omega, delta, C_Pf, C_Pr, vehicle_params, dt
        )

    return v_y, omega
