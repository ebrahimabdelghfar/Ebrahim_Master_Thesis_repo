from types import SimpleNamespace
from scipy.optimize import least_squares, minimize, differential_evolution
import numpy as np

from helpers.pacejka_formula import pacejka_formula

# scipy.optimize.least_squares methods: gradient-based, need a residual vector + bounds.
_LSQ_METHODS = ('trf', 'dogbox', 'lm')


def _robust_f_scale(F_y, cfg_f_scale):
    """Return robust inlier scale for robust least-squares loss."""
    if cfg_f_scale is not None:
        return float(cfg_f_scale)

    # Median absolute deviation based scale, clipped away from zero.
    med = np.median(F_y)
    mad = np.median(np.abs(F_y - med))
    return max(1e-3, 1.4826 * mad)


def _pacejka_sse(params, alpha, F_z, F_y):
    """Plain sum-of-squared-residuals objective for derivative-free/global solvers.

    Nelder-Mead and differential-evolution optimize a scalar, not a residual
    vector, so they can't use least_squares' soft_l1/f_scale robust loss -
    they always see the raw SSE.
    """
    r = pacejka_error(params, alpha, F_z, F_y)
    return float(np.dot(r, r))


def _fit_pacejka_axle(alpha, F_z, F_y, start_params, bounds, solver_cfg):
    """Fit one axle. method selects the optimizer family:

    trf/dogbox/lm       - scipy.optimize.least_squares (gradient-based, fast, local).
    nelder-mead         - scipy.optimize.minimize's derivative-free simplex search;
                          same solver arXiv:2603.09399 (this repo's own SOTA follow-on
                          paper) uses for iterative on-track Pacejka extraction.
    differential-evolution - scipy.optimize.differential_evolution, a global
                          population-based search; escapes local minima that
                          multi-start trf/nelder-mead can still miss, at higher
                          runtime cost. Same rationale this repo's own
                          pacejka_identification/coefficient_identifier.py
                          already applies via its genetic-algorithm identifier.

    trf/dogbox/lm/nelder-mead all reuse the lightweight multi-start (jitter)
    loop for local-minimum robustness; differential-evolution is already
    global and runs once.
    """
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

    if method == 'differential-evolution':
        res = differential_evolution(
            _pacejka_sse,
            bounds=list(zip(lb, ub)),
            args=(alpha, F_z, F_y),
            seed=seed,
            popsize=solver_cfg.get('de_popsize') or 15,
            maxiter=solver_cfg.get('de_maxiter') or 1000,
        )
        return SimpleNamespace(x=res.x)

    if method not in _LSQ_METHODS and method != 'nelder-mead':
        raise ValueError(f"Unknown pacejka_solver.method: {method!r}")

    rng = np.random.default_rng(seed)
    starts = [np.clip(base, lb, ub)]

    if n_starts > 1 and jitter > 0.0:
        span = np.maximum(np.abs(base), 1.0)
        for _ in range(n_starts - 1):
            candidate = base + rng.normal(0.0, jitter * span)
            starts.append(np.clip(candidate, lb, ub))

    best_x, best_cost = None, np.inf
    for x0 in starts:
        if method in _LSQ_METHODS:
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
            x, cost = res.x, res.cost
        else:  # nelder-mead
            res = minimize(
                _pacejka_sse,
                x0,
                args=(alpha, F_z, F_y),
                method='Nelder-Mead',
                bounds=list(zip(lb, ub)),
                options={'maxfev': max_nfev} if max_nfev else {},
            )
            x, cost = res.x, res.fun

        if cost < best_cost:
            best_x, best_cost = x, cost

    return SimpleNamespace(x=best_x)

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
        'de_popsize': model.get('pacejka_de_popsize', None),
        'de_maxiter': model.get('pacejka_de_maxiter', None),
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

