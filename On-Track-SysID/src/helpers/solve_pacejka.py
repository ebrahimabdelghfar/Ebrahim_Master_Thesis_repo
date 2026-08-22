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

    # Tikhonov pull toward the nominal coefficients, for the directions the
    # manoeuvre does not excite (see pacejka_error). 0 disables it.
    lam = float(solver_cfg.get('prior_weight', 0.0) or 0.0)
    prior_params = solver_cfg.get('prior_params', None)
    prior_base = base if prior_params is None else np.asarray(prior_params, dtype=float)
    prior = np.clip(prior_base, lb, ub) if lam > 0.0 else None
    prior_w = _prior_weights(lam, F_y, np.size(alpha), bounds) if lam > 0.0 else None
    args = (alpha, F_z, F_y, prior, prior_w)

    if method == 'differential-evolution':
        res = differential_evolution(
            _pacejka_sse,
            bounds=list(zip(lb, ub)),
            args=args,
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
                args=args,
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
                args=args,
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
    """Residual vector (not a summed scalar) for nonlinear least-squares.

    args is (alpha, F_z, F_y) or (alpha, F_z, F_y, prior, prior_weights).
    The optional prior block appends w_i * (p_i - prior_i) to the residual,
    i.e. Tikhonov regularisation toward the nominal coefficients / a Gaussian
    MAP estimate. It exists because not every Magic Formula coefficient is
    identifiable from every manoeuvre: E (curvature) in particular only shows
    up in the shape of the curve around its peak, so with data that stops
    short of the peak the cost surface is flat in E and the solver slides it
    onto a box edge - one of the three coefficients that were railed in
    bug-sysid-degenerate-pacejka-fit. Weighting is done by
    _prior_weights() so the term is scaled to the data residual, not to the
    raw parameter magnitudes.
    """
    alpha, F_z, F_y = args[0], args[1], args[2]
    y = pacejka_formula(params, alpha, F_z)
    r = y - F_y
    if len(args) >= 5 and args[3] is not None:
        prior, weights = args[3], args[4]
        r = np.concatenate([r, weights * (np.asarray(params, dtype=float) - prior)])
    return r


def _prior_weights(lam, F_y, n_samples, bounds):
    """Per-coefficient weights for the prior block of the residual vector.

    Each coefficient is normalised by its own box width so lam is a single
    dimensionless knob, and scaled by the data block's RMS residual budget
    (rms(F_y) * sqrt(n)) so lam has the same meaning regardless of how many
    samples or how much force the manoeuvre produced.
    """
    lb = np.asarray(bounds[0], dtype=float)
    ub = np.asarray(bounds[1], dtype=float)
    scale = np.sqrt(max(1, n_samples)) * max(1e-6, float(np.sqrt(np.mean(np.square(F_y)))))
    return lam * scale / (ub - lb)


# [B, C, D, E] bounds shared by both axles. Single-sourced here so any other
# module needing the D (peak-friction-coefficient) clip range - e.g. the
# friction warm-start estimator - imports this instead of duplicating it.
#
# These MUST stay in sync with adaptive_controller_manager.yaml's
# tire_param_min/tire_param_max ([Bf,Cf,Df,Ef,Br,Cr,Dr,Er]) - a fit that lands
# outside those is silently rejected by the manager and the MPC never gets it.
#
# Previous value was ([1.0, 0.1, 0.1, 0.0], [20.0, 20.0, 20.0, 5.0]). Those
# admitted parameter sets that are not valid Magic Formula at all, and the
# solver repeatedly converged onto the rails: observed fits included C=0.1,
# C=19.98, D=0.1, D=19.9989 and E=5.0 - every one of them exactly a bound.
# Feeding those to the MPC made its QP fail to solve (2026-08-16). Each bound
# below is a property of the formula Fy = D*Fz*sin(C*atan(B*a - E*(B*a - atan(B*a)))):
#
#   B  4.0 .. 20.0  stiffness factor; cornering stiffness is B*C*D*Fz.
#   C  1.2 ..  2.2  shape factor. MUST be > 1 or the lateral curve has no peak.
#                   C*atan() spans +/-C*pi/2, so C ~ 20 wraps the sine through
#                   ~5 periods - the "tire curve" then changes sign 6 times over
#                   alpha in [0, 0.4] rad and its Jacobian is meaningless.
#   D  0.4 ..  2.0  peak friction coefficient (Fy = D*Fz). For the CARLA
#                   asurt_fsai this should converge near 1.5, the PhysX
#                   wheel tire_friction set in carla_interface_config.yaml.
#   E -3.0 ..  1.0  curvature factor; the Magic Formula requires E <= 1, above
#                   which the argument folds back and the curve is non-physical.
PACEJKA_BOUNDS = ([4.0, 1.2, 0.4, -3.0], [20.0, 2.2, 2.0, 1.0])


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
        'prior_weight': model.get('pacejka_prior_weight', 0.0),
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

    # front. C_P*_prior, when present, is the regularisation target and stays
    # fixed across nn_train's co-identification iterations while C_P*_model
    # (the start point / rollout nominal) keeps updating - see nn_train.
    start_params_front = model['C_Pf_model']
    solver_cfg['prior_params'] = model.get('C_Pf_prior', None)
    sol_f = _fit_pacejka_axle(alpha_f, F_zf, F_yf, start_params_front, bounds, solver_cfg)
    C_Pf = sol_f.x.tolist()
    C_Pf = [round(x, 4) for x in C_Pf]  # Formatting each element to 4 significant digits

    # rear
    start_params_rear = model['C_Pr_model']
    solver_cfg['prior_params'] = model.get('C_Pr_prior', None)
    sol_r = _fit_pacejka_axle(alpha_r, F_zr, F_yr, start_params_rear, bounds, solver_cfg)
    C_Pr = sol_r.x.tolist()
    C_Pr = [round(x, 4) for x in C_Pr]  # Formatting each element to 4 significant digits
    return C_Pf, C_Pr

