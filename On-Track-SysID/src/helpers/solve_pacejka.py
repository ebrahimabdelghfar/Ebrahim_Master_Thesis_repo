from scipy.optimize import least_squares
import numpy as np


def _robust_f_scale(F_y, cfg_f_scale):
    """Return robust inlier scale for robust least-squares loss."""
    if cfg_f_scale is not None:
        return float(cfg_f_scale)

    # Median absolute deviation based scale, clipped away from zero.
    med = np.median(F_y)
    mad = np.median(np.abs(F_y - med))
    return max(1e-3, 1.4826 * mad)


def _fit_pacejka_axle(alpha, F_z, F_y, start_params, bounds, solver_cfg):
    """Fit one axle with optional lightweight multi-start for local-minimum robustness."""
    method = solver_cfg.get('method', 'trf')
    loss = solver_cfg.get('loss', 'soft_l1')
    x_scale = solver_cfg.get('x_scale', 'jac')
    max_nfev = solver_cfg.get('max_nfev', None)
    n_starts = int(max(1, solver_cfg.get('num_starts', 1)))
    jitter = float(max(0.0, solver_cfg.get('start_jitter', 0.05)))
    seed = solver_cfg.get('seed', None)
    f_scale = _robust_f_scale(F_y, solver_cfg.get('f_scale', None))

    lb = np.asarray(bounds[0], dtype=float)
    ub = np.asarray(bounds[1], dtype=float)
    base = np.asarray(start_params, dtype=float)

    rng = np.random.default_rng(seed)
    starts = [np.clip(base, lb, ub)]

    if n_starts > 1 and jitter > 0.0:
        span = np.maximum(np.abs(base), 1.0)
        for _ in range(n_starts - 1):
            candidate = base + rng.normal(0.0, jitter * span)
            starts.append(np.clip(candidate, lb, ub))

    best_res = None
    for x0 in starts:
        res = least_squares(
            pacejka_error,
            x0,
            args=(alpha, F_z, F_y),
            bounds=bounds,
            method=method,
            loss=loss,
            f_scale=f_scale,
            x_scale=x_scale,
            max_nfev=max_nfev,
        )

        if (best_res is None) or (res.cost < best_res.cost):
            best_res = res

    return best_res

def analyse_tires(model, v_x, v_y, omega, delta):
    delta = delta.copy()
    v_y = v_y.copy()
    omega = omega.copy()
    v_x = v_x.copy()
    l_f = model['l_f']
    l_r = model['l_r']
    l_wb = model['l_wb']
    m = model['m']
    g_ = 9.81
    F_zf = m * g_ * l_r / l_wb
    F_zr = m * g_ * l_f / l_wb
    
    alpha_f = -np.arctan((v_y + omega * l_f) / v_x) + delta
    alpha_r = -np.arctan((v_y - omega * l_r) / v_x)

    F_yf = m * l_r * v_x * omega / ((l_r + l_f) * np.cos(delta))
    F_yr = m * l_f * v_x * omega / (l_r + l_f)
    
    # Filtering out diverged data points
    keep_idxs = np.where((abs(omega) <= 5))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]

    keep_idxs = np.where((abs(alpha_f) <= 0.5))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]

    keep_idxs = np.where((abs(alpha_r) <= 0.2))
    v_x = v_x[keep_idxs]
    v_y = v_y[keep_idxs]
    omega = omega[keep_idxs]
    delta = delta[keep_idxs]
    alpha_f = alpha_f[keep_idxs]
    alpha_r = alpha_r[keep_idxs]
    F_yf = F_yf[keep_idxs]
    F_yr = F_yr[keep_idxs]

    return alpha_f, alpha_r, F_zf, F_zr, F_yf, F_yr


def pacejka_formula(params, alpha, F_z):
    B, C, D, E = params
    y =  F_z * D * np.sin(C*np.arctan(B*alpha - E * (B*alpha -np.arctan(B * alpha))))
    return y

def pacejka_error(params, *args):
    alpha, F_z, F_y = args
    y = pacejka_formula(params, alpha, F_z)
    # Return residual vector (not summed scalar) for nonlinear least-squares.
    return y - F_y


# [B, C, D, E] bounds shared by both axles. Single-sourced here so any other
# module needing the D (peak-friction-coefficient) clip range - e.g. the
# friction warm-start estimator - imports this instead of duplicating it.
PACEJKA_BOUNDS = ([1.0, 0.1, 0.1, 0.0], [20.0, 20.0, 20.0, 5.0])


def solve_pacejka(model, v_x, v_y, omega, delta):
    alpha_f, alpha_r, F_zf, F_zr, F_yf, F_yr = analyse_tires(model, v_x, v_y, omega, delta)
    bounds = PACEJKA_BOUNDS

    # Optional solver tuning through model dict; defaults remain backward-compatible.
    solver_cfg = {
        'method': model.get('pacejka_method', 'trf'),
        'loss': model.get('pacejka_loss', 'soft_l1'),
        'f_scale': model.get('pacejka_f_scale', None),
        'x_scale': model.get('pacejka_x_scale', 'jac'),
        'max_nfev': model.get('pacejka_max_nfev', None),
        'num_starts': model.get('pacejka_num_starts', 1),
        'start_jitter': model.get('pacejka_start_jitter', 0.05),
        'seed': model.get('pacejka_seed', None),
    }

    # Fallback to previous coefficients when the filtered data is too sparse.
    if alpha_f.size < 8 or alpha_r.size < 8:
        C_Pf = [round(float(x), 4) for x in model['C_Pf_model']]
        C_Pr = [round(float(x), 4) for x in model['C_Pr_model']]
        return C_Pf, C_Pr

    # front
    start_params_front = model['C_Pf_model']
    sol_f = _fit_pacejka_axle(alpha_f, F_zf, F_yf, start_params_front, bounds, solver_cfg)
    C_Pf = sol_f.x.tolist()
    C_Pf = [round(x, 4) for x in C_Pf]  # Formatting each element to 4 significant digits

    # rear
    start_params_rear = model['C_Pr_model']
    sol_r = _fit_pacejka_axle(alpha_r, F_zr, F_yr, start_params_rear, bounds, solver_cfg)
    C_Pr = sol_r.x.tolist()
    C_Pr = [round(x, 4) for x in C_Pr]  # Formatting each element to 4 significant digits
    return C_Pf, C_Pr

