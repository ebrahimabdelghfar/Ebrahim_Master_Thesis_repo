#!/usr/bin/env python3
"""
Pacejka Magic Formula coefficient identifier.

Provides two identification strategies:
  1. simultaneous  – fit all [B, C, D, E] at once (fast, but B/C/E are correlated)
  2. sequential    – physics-grounded: fix C, fix D from the data peak, fit B & E
                     (breaks parameter degeneracy → unique, physically meaningful coefficients)

For ground-truth identification the *sequential* method is recommended: it
constrains C to a physically meaningful value, reads D directly off the
measured peak, and only leaves B (stiffness) and E (curvature) free. This
avoids the well-known parameter non-uniqueness problem of the Magic Formula
where different (B, C, D, E) combinations produce nearly identical curves.

Supported optimisers for the fitting step:
  - trust_region             (scipy.optimize.least_squares, TRF)
  - differential_evolution   (scipy.optimize.differential_evolution, global)
  - dual                     (DE → TRF, best accuracy)
  - genetic_algorithm        (custom real-coded GA, global)
  - ga_trust_region          (GA → TRF, hybrid global-local)
  - adaptive_de              (JADE — self-adaptive F/CR, global)
  - adaptive_de_trust_region (JADE → TRF, hybrid global-local)
  - bayesian_svi             (Stochastic Variational Inference via Pyro,
                               correlated posterior + parameter uncertainty)

Simultaneous mode additionally supports optional MAP regularization
(`regularization='map'`): tighter, literature-consistent bounds plus a
Gaussian-prior penalty anchoring C toward its physically-typical value, so
the optimizer can't drift into unphysical (B, C, D, E) combinations that
still happen to minimise the raw sum-of-squares on noisy/imbalanced data.
This — together with the optional slip-bin data rebalancing in `identify()`
— follows Goblirsch et al., "Bayesian Optimization-based Tire Parameter and
Uncertainty Estimation for Real-World Data", TUM, arXiv:2504.20863 (2025),
which uses physical parameter bounds, a correlated multivariate-normal
posterior fit via SVI/ELBO, and rebalanced slip-angle sampling to avoid
overfitting the (densely sampled) linear region.

Reference: Pacejka, H.B. "Tire and Vehicle Dynamics", 3rd Ed., Ch. 4.
"""

import numpy as np
from scipy.optimize import least_squares, differential_evolution

from pacejka_identification.magic_formula import pacejka_formula


# ═══════════════════════════════════════════════════════════════════════════
# Genetic Algorithm Engine
# ═══════════════════════════════════════════════════════════════════════════

class GeneticAlgorithm:
    """
    Real-coded Genetic Algorithm for Pacejka coefficient identification.

    Features:
      - Tournament selection
      - BLX-α crossover (blend crossover for real-valued parameters)
      - Gaussian mutation with adaptive step size
      - Elitism (top individuals survive to next generation)

    Parameters
    ----------
    cost_fn : callable(params, *args) → float
        Objective function to minimise.
    bounds : list of (lower, upper)
        Parameter bounds.
    args : tuple
        Extra arguments passed to cost_fn.
    pop_size : int
        Population size.
    n_generations : int
        Number of generations.
    crossover_rate : float
        Probability of crossover.
    mutation_rate : float
        Probability per gene mutation.
    mutation_scale : float
        Std-dev of Gaussian mutation as fraction of range.
    elite_frac : float
        Fraction of population preserved via elitism.
    tournament_size : int
        Number of individuals competing in selection.
    seed : int or None
        Random seed for reproducibility.
    """

    def __init__(
        self,
        cost_fn,
        bounds,
        args=(),
        pop_size=100,
        n_generations=300,
        crossover_rate=0.85,
        mutation_rate=0.15,
        mutation_scale=0.1,
        elite_frac=0.05,
        tournament_size=3,
        seed=42,
    ):
        self.cost_fn = cost_fn
        self.bounds = np.array(bounds, dtype=np.float64)  # shape (n_params, 2)
        self.args = args
        self.n_params = len(bounds)
        self.pop_size = pop_size
        self.n_gen = n_generations
        self.cx_rate = crossover_rate
        self.mut_rate = mutation_rate
        self.mut_scale = mutation_scale
        self.n_elite = max(1, int(elite_frac * pop_size))
        self.tour_size = tournament_size
        self.rng = np.random.default_rng(seed)

    def run(self):
        """Execute the GA and return the best solution."""
        lb = self.bounds[:, 0]
        ub = self.bounds[:, 1]
        ranges = ub - lb

        # Initialise population uniformly within bounds
        pop = self.rng.uniform(lb, ub, size=(self.pop_size, self.n_params))
        fitness = np.array([self.cost_fn(ind, *self.args) for ind in pop])

        best_idx = np.argmin(fitness)
        best_sol = pop[best_idx].copy()
        best_cost = fitness[best_idx]

        for gen in range(self.n_gen):
            # Sort by fitness
            order = np.argsort(fitness)
            pop = pop[order]
            fitness = fitness[order]

            # Elitism — keep top individuals
            new_pop = list(pop[:self.n_elite])

            # Fill remainder with crossover + mutation
            while len(new_pop) < self.pop_size:
                # Tournament selection — two parents
                p1 = self._tournament(pop, fitness)
                p2 = self._tournament(pop, fitness)

                # BLX-α crossover
                if self.rng.random() < self.cx_rate:
                    child = self._blx_crossover(p1, p2, lb, ub)
                else:
                    child = p1.copy() if self.rng.random() < 0.5 else p2.copy()

                # Gaussian mutation
                child = self._mutate(child, ranges, lb, ub)

                new_pop.append(child)

            pop = np.array(new_pop[:self.pop_size])
            fitness = np.array([self.cost_fn(ind, *self.args) for ind in pop])

            gen_best = np.argmin(fitness)
            if fitness[gen_best] < best_cost:
                best_cost = fitness[gen_best]
                best_sol = pop[gen_best].copy()

        return best_sol, best_cost

    def _tournament(self, pop, fitness):
        """Tournament selection: pick best of random subset."""
        idxs = self.rng.integers(0, len(pop), size=self.tour_size)
        winner = idxs[np.argmin(fitness[idxs])]
        return pop[winner].copy()

    def _blx_crossover(self, p1, p2, lb, ub, alpha=0.5):
        """BLX-α blend crossover for real-valued chromosomes."""
        d = np.abs(p1 - p2)
        lo = np.minimum(p1, p2) - alpha * d
        hi = np.maximum(p1, p2) + alpha * d
        child = self.rng.uniform(lo, hi)
        return np.clip(child, lb, ub)

    def _mutate(self, child, ranges, lb, ub):
        """Per-gene Gaussian mutation."""
        for i in range(self.n_params):
            if self.rng.random() < self.mut_rate:
                sigma = self.mut_scale * ranges[i]
                child[i] += self.rng.normal(0, sigma)
        return np.clip(child, lb, ub)


# ═══════════════════════════════════════════════════════════════════════════
# JADE — Adaptive Differential Evolution (Zhang & Sanderson, 2009)
# ═══════════════════════════════════════════════════════════════════════════

class JADE:
    """
    JADE: Adaptive Differential Evolution with optional external archive.

    Reference:
        J. Zhang and A.C. Sanderson, "JADE: Adaptive Differential Evolution
        with Optional External Archive", IEEE Trans. Evol. Comput., 2009.

    Key innovations over standard DE:
      1. **current-to-pbest/1 mutation** — uses the top-p fraction of the
         population rather than the single best, improving diversity.
      2. **Self-adaptive F and CR** — each individual draws its own F from
         a Cauchy distribution and CR from a Normal distribution.  After
         each generation the location parameters (μF, μCR) are updated
         using only the *successful* F/CR values (Lehmer mean for F,
         arithmetic mean for CR).
      3. **External archive** — replaced (inferior) individuals are stored
         in an archive and can be used during mutation, further increasing
         exploration.

    Parameters
    ----------
    cost_fn : callable(params, *args) → float
    bounds  : list of (lower, upper)
    args    : tuple
    pop_size      : int   — population size (NP)
    n_generations : int   — maximum generations
    p             : float — top-p fraction for pbest selection (0 < p ≤ 1)
    c             : float — adaptation rate for μF, μCR  (0 < c ≤ 1)
    archive_ratio : float — max archive size as fraction of pop_size
    seed          : int or None
    """

    def __init__(
        self,
        cost_fn,
        bounds,
        args=(),
        pop_size=100,
        n_generations=400,
        p=0.1,
        c=0.1,
        archive_ratio=1.0,
        seed=42,
    ):
        self.cost_fn = cost_fn
        self.bounds = np.array(bounds, dtype=np.float64)
        self.args = args
        self.n_params = len(bounds)
        self.NP = pop_size
        self.n_gen = n_generations
        self.p = max(p, 2.0 / pop_size)  # at least 2 individuals in pbest
        self.c = c
        self.max_archive = int(archive_ratio * pop_size)
        self.rng = np.random.default_rng(seed)

    def run(self):
        lb = self.bounds[:, 0]
        ub = self.bounds[:, 1]

        # Initialise population
        pop = self.rng.uniform(lb, ub, size=(self.NP, self.n_params))
        fitness = np.array([self.cost_fn(ind, *self.args) for ind in pop])

        # Adaptive location parameters
        mu_CR = 0.5
        mu_F = 0.5

        # External archive
        archive = []

        best_idx = np.argmin(fitness)
        best_sol = pop[best_idx].copy()
        best_cost = fitness[best_idx]

        for gen in range(self.n_gen):
            S_CR = []  # successful CR values this generation
            S_F = []   # successful F values this generation

            trial_pop = np.empty_like(pop)

            # pbest pool is fixed for the whole generation (fitness only
            # changes after selection below) — compute once, not per-individual.
            n_pbest = max(2, int(self.p * self.NP))
            pbest_indices = np.argsort(fitness)[:n_pbest]

            for i in range(self.NP):
                # --- Generate CR_i ~ N(mu_CR, 0.1), clipped to [0, 1] ---
                CR_i = np.clip(self.rng.normal(mu_CR, 0.1), 0.0, 1.0)

                # --- Generate F_i ~ Cauchy(mu_F, 0.1), truncated to (0, 1] ---
                F_i = self._rand_cauchy(mu_F, 0.1)

                # --- current-to-pbest/1 mutation ---
                pbest = pop[self.rng.choice(pbest_indices)]

                # Select r1 != i from population
                r1 = i
                while r1 == i:
                    r1 = self.rng.integers(0, self.NP)

                # Select r2 != i, r1 from population ∪ archive
                pool = list(range(self.NP)) + list(range(self.NP, self.NP + len(archive)))
                r2 = i
                while r2 == i or r2 == r1:
                    r2_idx = self.rng.integers(0, len(pool))
                    r2 = pool[r2_idx]

                x_r2 = pop[r2] if r2 < self.NP else archive[r2 - self.NP]

                # Mutation: v = x_i + F*(x_pbest - x_i) + F*(x_r1 - x_r2)
                v = pop[i] + F_i * (pbest - pop[i]) + F_i * (pop[r1] - x_r2)
                v = np.clip(v, lb, ub)

                # --- Binomial crossover ---
                j_rand = self.rng.integers(0, self.n_params)
                mask = (self.rng.random(self.n_params) < CR_i)
                mask[j_rand] = True  # ensure at least one parameter from mutant
                trial = np.where(mask, v, pop[i])

                trial_pop[i] = trial

            # --- Selection ---
            trial_fitness = np.array([self.cost_fn(t, *self.args) for t in trial_pop])

            for i in range(self.NP):
                if trial_fitness[i] <= fitness[i]:
                    # Store replaced individual in archive
                    if len(archive) < self.max_archive:
                        archive.append(pop[i].copy())
                    else:
                        # Replace random archive member
                        idx = self.rng.integers(0, self.max_archive)
                        archive[idx] = pop[i].copy()

                    pop[i] = trial_pop[i]
                    fitness[i] = trial_fitness[i]
                    S_CR.append(CR_i)
                    S_F.append(F_i)

            # --- Update μCR and μF using successful values ---
            if S_CR and S_F:
                mu_CR = (1 - self.c) * mu_CR + self.c * np.mean(S_CR)
                # Lehmer mean for F (weighted toward larger values)
                S_F = np.array(S_F)
                mu_F = (1 - self.c) * mu_F + self.c * (np.sum(S_F ** 2) / np.sum(S_F))

            # Track global best
            gen_best = np.argmin(fitness)
            if fitness[gen_best] < best_cost:
                best_cost = fitness[gen_best]
                best_sol = pop[gen_best].copy()

        return best_sol, best_cost

    def _rand_cauchy(self, loc, scale):
        """Draw from Cauchy(loc, scale), truncated to (0, 1]."""
        while True:
            x = loc + scale * np.tan(np.pi * (self.rng.random() - 0.5))
            if 0 < x <= 1:
                return x


# ═══════════════════════════════════════════════════════════════════════════
# Residual / cost helpers
# ═══════════════════════════════════════════════════════════════════════════

def _residuals(params, slip, fz, y_meas):
    """Element-wise residual vector: model − measured."""
    return pacejka_formula(params, slip, fz) - y_meas


def _cost(params, slip, fz, y_meas):
    """Sum-of-squares cost (scalar) for DE."""
    return float(np.sum(_residuals(params, slip, fz, y_meas) ** 2))


def _residuals_BE(be_params, C, D, slip, fz, y_meas):
    """Residual with C and D fixed; only B and E are free."""
    B, E = be_params
    return pacejka_formula([B, C, D, E], slip, fz) - y_meas


def _cost_BE(be_params, C, D, slip, fz, y_meas):
    """Scalar cost with C and D fixed."""
    return float(np.sum(_residuals_BE(be_params, C, D, slip, fz, y_meas) ** 2))


def _residuals_reg(params, slip, fz, y_meas, C_lit, lambda_C, sigma_C):
    """
    Residual vector for simultaneous MAP regularisation: the ordinary
    residuals plus one extra pseudo-residual penalising deviation of C from
    its literature-typical value. Minimising sum-of-squares of this vector
    is equivalent to minimising SSE(params) + lambda_C*((C-C_lit)/sigma_C)^2,
    i.e. a Gaussian-prior MAP estimate for C.
    """
    res = _residuals(params, slip, fz, y_meas)
    C = params[1]
    reg = np.sqrt(max(lambda_C, 0.0)) * (C - C_lit) / max(sigma_C, 1e-9)
    return np.concatenate([res, [reg]])


def _cost_reg(params, slip, fz, y_meas, C_lit, lambda_C, sigma_C):
    """Scalar MAP-regularised cost (SSE + Gaussian-prior penalty on C)."""
    return float(np.sum(_residuals_reg(params, slip, fz, y_meas, C_lit, lambda_C, sigma_C) ** 2))


# ═══════════════════════════════════════════════════════════════════════════
# Fit-quality metrics
# ═══════════════════════════════════════════════════════════════════════════

def _metrics(params, slip, fz, y_meas):
    y_pred = pacejka_formula(params, slip, fz)
    res = y_meas - y_pred
    n = len(y_meas)
    ss_res = np.sum(res ** 2)
    ss_tot = np.sum((y_meas - np.mean(y_meas)) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')
    rmse = np.sqrt(ss_res / n) if n > 0 else float('nan')
    mae = np.mean(np.abs(res)) if n > 0 else float('nan')
    max_ae = float(np.max(np.abs(res))) if n > 0 else float('nan')
    return {
        'R2': round(float(r2), 6),
        'RMSE': round(float(rmse), 4),
        'MAE': round(float(mae), 4),
        'MaxAE': round(float(max_ae), 4),
        'n_samples': int(n),
        'residual_norm': round(float(np.sqrt(ss_res)), 4),
    }


# ═══════════════════════════════════════════════════════════════════════════
# Sequential identification helpers
# ═══════════════════════════════════════════════════════════════════════════

def _estimate_D_from_peak(slip, fz, y_meas, n_bins=50):
    """
    Estimate the peak factor D from data.

    D ≈ max|Y / Fz|  (the normalised peak force/torque). A raw per-sample
    percentile inherits the full measurement-noise variance of whichever
    single point happens to rank highest, which the "sequential" fit is
    especially sensitive to: since D is now *fixed* rather than floated
    (breaking the B-C-D-E degeneracy — see `_sequential`), any offset in D
    gets absorbed almost entirely by E during the B,E fit (empirically,
    ~15-20x amplification for this Magic Formula shape near the peak).
    So instead of ranking raw samples, bin the slip range and take the peak
    of the *bin means* — this averages out per-sample noise before the peak
    is read off, at the cost of a small, well-controlled window-averaging
    bias (negligible near a true maximum, where the curve is locally flat).
    """
    fz_safe = np.where(np.abs(fz) > 1.0, fz, 1.0)
    y_norm = np.abs(y_meas / fz_safe)

    lo, hi = np.min(slip), np.max(slip)
    if hi <= lo or len(slip) < n_bins:
        return float(np.percentile(y_norm, 99))  # too little data to bin usefully

    edges = np.linspace(lo, hi, n_bins + 1)
    bin_idx = np.clip(np.digitize(slip, edges[1:-1]), 0, n_bins - 1)
    bin_means = [y_norm[bin_idx == b].mean() for b in range(n_bins) if np.any(bin_idx == b)]
    return float(np.max(bin_means))


def _estimate_cornering_stiffness(slip, fz, y_meas, window=0.02):
    """
    Estimate the cornering stiffness BCD from linear region data.

    BCD ≈ slope of (Y/Fz) vs slip near slip=0.
    """
    mask = np.abs(slip) < window
    if np.sum(mask) < 5:
        mask = np.abs(slip) < 0.05  # fallback wider window
    if np.sum(mask) < 5:
        return None  # not enough data near zero

    fz_safe = np.where(np.abs(fz[mask]) > 1.0, fz[mask], 1.0)
    y_norm = y_meas[mask] / fz_safe
    s = slip[mask]

    # Linear regression: y_norm = BCD * s + intercept
    A = np.vstack([s, np.ones_like(s)]).T
    result = np.linalg.lstsq(A, y_norm, rcond=None)
    bcd = result[0][0]
    return float(bcd)


def _rebalance_slip_bins(slip, fz, y_meas, n_bins=20, max_per_bin=200, seed=42):
    """
    Subsample over-represented slip bins so a dense near-zero-slip cluster
    (typical of real driving data) doesn't dominate the least-squares fit at
    the expense of the sparser near-peak region. Follows Goblirsch et al.
    (arXiv:2504.20863), who rebalance slip-angle sampling density "to
    prevent overfitting of the linear region".
    """
    n = len(slip)
    if n == 0 or n_bins < 1:
        return slip, fz, y_meas

    rng = np.random.default_rng(seed)
    lo, hi = np.min(slip), np.max(slip)
    if hi <= lo:
        return slip, fz, y_meas

    edges = np.linspace(lo, hi, n_bins + 1)
    bin_idx = np.clip(np.digitize(slip, edges[1:-1]), 0, n_bins - 1)

    keep = []
    for b in range(n_bins):
        idx = np.where(bin_idx == b)[0]
        if len(idx) > max_per_bin:
            idx = rng.choice(idx, size=max_per_bin, replace=False)
        keep.append(idx)
    keep = np.sort(np.concatenate(keep))

    return slip[keep], fz[keep], y_meas[keep]


# ═══════════════════════════════════════════════════════════════════════════
# Main class
# ═══════════════════════════════════════════════════════════════════════════

# Default C values grounded in Pacejka literature
DEFAULT_C = {
    'fy': 1.30,    # lateral force shape factor (typical 1.1 – 1.8)
    'fx': 1.65,    # longitudinal force shape factor (typical 1.4 – 1.8)
    'mz': 2.40,    # self-aligning torque shape factor (typical 2.0 – 3.0)
}

# Physically-informed bounds for [B, C, D, E], used only when
# `regularization='map'` (simultaneous mode). Fy/Fx ranges are taken
# directly from Goblirsch et al., "Bayesian Optimization-based Tire
# Parameter and Uncertainty Estimation for Real-World Data", TUM,
# arXiv:2504.20863 (2025), Table I. That paper only covers lateral force;
# Mz's normalised peak (self-aligning torque / Fz) is roughly an order of
# magnitude smaller than Fy/Fx in this package's own reference values (e.g.
# its own test fixtures use D_mz≈0.05 vs. D_fy≈0.69/D_fx≈1.1), so its D
# bound is scaled down proportionally rather than sourced from the paper.
LITERATURE_BOUNDS = {
    'fy': ([5.0, 1.0, 0.1, -1.0], [40.0, 3.0, 2.0, 1.0]),
    'fx': ([5.0, 1.0, 0.1, -1.0], [40.0, 3.0, 2.0, 1.0]),
    'mz': ([5.0, 1.0, 0.01, -1.0], [40.0, 3.0, 0.3, 1.0]),
}


class CoefficientIdentifier:
    """
    Identifies Pacejka [B, C, D, E] from measured (slip, Fz, force) data.

    Parameters
    ----------
    method : str
        Fitting strategy: 'trust_region', 'differential_evolution', 'dual',
        'genetic_algorithm', 'ga_trust_region', 'adaptive_de',
        'adaptive_de_trust_region', or 'bayesian_svi'.
    identification_mode : str
        'sequential' (recommended, fixes C and D) or 'simultaneous' (fits all 4).
    fixed_C : dict or None
        Override default C values, e.g. {'fy': 1.3, 'fx': 1.65, 'mz': 2.4}.
    lower_bounds / upper_bounds : list
        Bounds for [B, C, D, E] (simultaneous, when regularization='none')
        or [B, D, E] (sequential — only lb/ub[0]/[2]/[3] are used).
    tr_params : dict or None
        Trust-Region hyperparameters: {max_nfev}.
    de_params : dict or None
        Differential Evolution hyperparameters: {maxiter, tol, seed, polish}.
    ga_params : dict or None
        Genetic Algorithm hyperparameters:
        {pop_size, n_generations, crossover_rate, mutation_rate,
         mutation_scale, elite_frac, tournament_size, seed}.
    jade_params : dict or None
        JADE hyperparameters: {pop_size, n_generations, p, c, archive_ratio, seed}.
    regularization : str
        'none' (default) or 'map' — simultaneous-mode only. 'map' switches
        to literature-consistent bounds (`LITERATURE_BOUNDS`) and adds a
        Gaussian-prior penalty anchoring C toward `DEFAULT_C`/`fixed_C`.
    map_reg_params : dict or None
        {'lambda_C', 'sigma_C'} — MAP penalty weight and prior std-dev for C.
    svi_params : dict or None
        {'num_steps', 'learning_rate', 'seed'} — hyperparameters for the
        `bayesian_svi` method.
    data_balancing_params : dict or None
        {'enabled', 'n_bins', 'max_per_bin'} — optional slip-histogram
        rebalancing applied in `identify()` before fitting (both modes).
    """

    # Default hyperparameters for each algorithm
    _DEFAULT_TR = {'max_nfev': 10000}
    _DEFAULT_DE = {'maxiter': 1000, 'tol': 1e-12, 'seed': 42, 'polish': True}
    _DEFAULT_GA = {
        'pop_size': 120, 'n_generations': 400,
        'crossover_rate': 0.85, 'mutation_rate': 0.15,
        'mutation_scale': 0.10, 'elite_frac': 0.05,
        'tournament_size': 3, 'seed': 42,
    }
    _DEFAULT_JADE = {
        'pop_size': 100, 'n_generations': 400,
        'p': 0.1, 'c': 0.1, 'archive_ratio': 1.0, 'seed': 42,
    }
    _DEFAULT_MAP_REG = {'lambda_C': 1.0, 'sigma_C': 0.3}
    _DEFAULT_SVI = {'num_steps': 2000, 'learning_rate': 0.01, 'seed': 42}
    _DEFAULT_BALANCING = {'enabled': False, 'n_bins': 20, 'max_per_bin': 200}

    def __init__(
        self,
        method='dual',
        identification_mode='sequential',
        fixed_C=None,
        lower_bounds=None,
        upper_bounds=None,
        tr_params=None,
        de_params=None,
        ga_params=None,
        jade_params=None,
        regularization='none',
        map_reg_params=None,
        svi_params=None,
        data_balancing_params=None,
    ):
        allowed_methods = {
            'trust_region', 'differential_evolution', 'dual',
            'genetic_algorithm', 'ga_trust_region',
            'adaptive_de', 'adaptive_de_trust_region',
            'bayesian_svi',
        }
        if method not in allowed_methods:
            raise ValueError(f"method must be one of {allowed_methods}")

        allowed_modes = {'sequential', 'simultaneous'}
        if identification_mode not in allowed_modes:
            raise ValueError(f"identification_mode must be one of {allowed_modes}")

        allowed_reg = {'none', 'map'}
        if regularization not in allowed_reg:
            raise ValueError(f"regularization must be one of {allowed_reg}")

        self.method = method
        self.id_mode = identification_mode
        self.regularization = regularization

        # C values for sequential mode / MAP prior mean
        self.C_values = dict(DEFAULT_C)
        if fixed_C:
            self.C_values.update(fixed_C)

        # Bounds — full [B, C, D, E]
        self.lb = np.array(lower_bounds if lower_bounds else [0.1, 0.1, 0.01, -2.0])
        self.ub = np.array(upper_bounds if upper_bounds else [50.0, 5.0, 5.0, 2.0])

        # Algorithm hyperparameters (merge user overrides onto defaults)
        self.tr = {**self._DEFAULT_TR, **(tr_params or {})}
        self.de = {**self._DEFAULT_DE, **(de_params or {})}
        self.ga = {**self._DEFAULT_GA, **(ga_params or {})}
        self.jade = {**self._DEFAULT_JADE, **(jade_params or {})}
        self.map_reg = {**self._DEFAULT_MAP_REG, **(map_reg_params or {})}
        self.svi = {**self._DEFAULT_SVI, **(svi_params or {})}
        self.data_balancing = {**self._DEFAULT_BALANCING, **(data_balancing_params or {})}

        # Handle seed=-1 → None (non-deterministic)
        if self.de['seed'] == -1:
            self.de['seed'] = None
        if self.ga['seed'] == -1:
            self.ga['seed'] = None
        if self.jade['seed'] == -1:
            self.jade['seed'] = None
        if self.svi['seed'] == -1:
            self.svi['seed'] = None

        # Set by _svi_all/_svi_be for the current identify() call, merged
        # into the returned metrics dict as *_std uncertainty fields.
        self._last_uncertainty = None

    # ──────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────

    def identify(self, slip, fz, y_meas, initial_guess=None, label=''):
        slip = np.asarray(slip, dtype=np.float64)
        fz = np.asarray(fz, dtype=np.float64)
        y_meas = np.asarray(y_meas, dtype=np.float64)

        if self.data_balancing.get('enabled', False):
            slip, fz, y_meas = _rebalance_slip_bins(
                slip, fz, y_meas,
                n_bins=self.data_balancing['n_bins'],
                max_per_bin=self.data_balancing['max_per_bin'],
            )

        if initial_guess is None:
            initial_guess = [10.0, 1.5, 1.0, 0.5]

        self._last_uncertainty = None

        if self.id_mode == 'sequential':
            coeffs = self._sequential(slip, fz, y_meas, initial_guess, label)
        else:
            coeffs = self._simultaneous(slip, fz, y_meas, initial_guess, label)

        met = _metrics(coeffs, slip, fz, y_meas)
        if self._last_uncertainty:
            met.update(self._last_uncertainty)
        coeffs_list = [round(float(c), 6) for c in coeffs]
        return coeffs_list, met

    def identify_fy(self, alpha, fz, fy, initial_guess=None):
        return self.identify(alpha, fz, fy, initial_guess, label='fy')

    def identify_fx(self, kappa, fz, fx, initial_guess=None):
        return self.identify(kappa, fz, fx, initial_guess, label='fx')

    def identify_mz(self, alpha, fz, mz, initial_guess=None):
        return self.identify(alpha, fz, mz, initial_guess, label='mz')

    @staticmethod
    def _channel_from_label(label):
        """Map a free-form label (e.g. 'longitudinal_fx') to 'fy'/'fx'/'mz'."""
        label_lower = label.lower()
        if 'fx' in label_lower or 'longitudinal' in label_lower:
            return 'fx'
        elif 'mz' in label_lower or 'aligning' in label_lower:
            return 'mz'
        return 'fy'

    def _literature_bounds(self, channel):
        lb, ub = LITERATURE_BOUNDS[channel]
        return np.array(lb, dtype=np.float64), np.array(ub, dtype=np.float64)

    # ──────────────────────────────────────────────────────────────────
    # Generic optimiser back-ends (shared by sequential's [B,E] fit and
    # simultaneous's [B,C,D,E] fit — the only difference is which
    # residuals/cost function and bounds are passed in).
    # ──────────────────────────────────────────────────────────────────

    def _tr_fit(self, residuals_fn, x0, args, lb, ub):
        res = least_squares(
            residuals_fn, x0, args=args,
            bounds=(lb, ub), method='trf',
            max_nfev=self.tr['max_nfev'],
        )
        return res.x

    def _de_fit(self, cost_fn, args, lb, ub):
        bounds_list = list(zip(lb, ub))
        res = differential_evolution(
            cost_fn, bounds=bounds_list, args=args,
            seed=self.de['seed'], maxiter=self.de['maxiter'],
            tol=self.de['tol'], polish=self.de['polish'],
        )
        return res.x

    def _ga_fit(self, cost_fn, args, lb, ub):
        bounds_list = list(zip(lb, ub))
        ga = GeneticAlgorithm(
            cost_fn=cost_fn,
            bounds=bounds_list,
            args=args,
            pop_size=self.ga['pop_size'],
            n_generations=self.ga['n_generations'],
            crossover_rate=self.ga['crossover_rate'],
            mutation_rate=self.ga['mutation_rate'],
            mutation_scale=self.ga['mutation_scale'],
            elite_frac=self.ga['elite_frac'],
            tournament_size=self.ga['tournament_size'],
            seed=self.ga['seed'],
        )
        best_sol, _ = ga.run()
        return best_sol

    def _jade_fit(self, cost_fn, args, lb, ub):
        bounds_list = list(zip(lb, ub))
        jade = JADE(
            cost_fn=cost_fn,
            bounds=bounds_list,
            args=args,
            pop_size=self.jade['pop_size'],
            n_generations=self.jade['n_generations'],
            p=self.jade['p'],
            c=self.jade['c'],
            archive_ratio=self.jade['archive_ratio'],
            seed=self.jade['seed'],
        )
        best_sol, _ = jade.run()
        return best_sol

    # ──────────────────────────────────────────────────────────────────
    # Sequential identification (recommended)
    # ──────────────────────────────────────────────────────────────────

    def _sequential(self, slip, fz, y_meas, x0, label):
        """
        Physics-grounded sequential identification:
          1. Fix C to a literature-based value for the force channel.
          2. Fix D to the (bounds-clipped) 99th-percentile peak estimate
             from the data — not merely seed it, so it can't trade off
             against B/E during optimisation.
          3. Seed B from the cornering-stiffness slope BCD/(C·D) near the
             origin.
          4. Fit only B, E (with C, D fixed) using the chosen optimiser.
        This breaks the Magic Formula's parameter degeneracy and produces
        unique, physically meaningful coefficients.
        """
        channel = self._channel_from_label(label)
        C = self.C_values[channel]

        B0, D0, E0 = x0[0], x0[2], x0[3]

        # Fix D from the data peak (clipped to the configured D bounds,
        # not the unrelated 0.001/10.0 window used previously).
        D_est = _estimate_D_from_peak(slip, fz, y_meas)
        D_lb, D_ub = float(self.lb[2]), float(self.ub[2])
        if D_est is not None and np.isfinite(D_est):
            D0 = D_est
        D = float(np.clip(D0, D_lb, D_ub))

        # Seed B from the cornering-stiffness slope near the origin.
        bcd_est = _estimate_cornering_stiffness(slip, fz, y_meas)
        if bcd_est is not None and C * D > 1e-9:
            B0 = abs(bcd_est) / (C * D)
        B0 = float(np.clip(B0, self.lb[0], self.ub[0]))

        lb_be = np.array([self.lb[0], self.lb[3]])
        ub_be = np.array([self.ub[0], self.ub[3]])
        be0 = np.array([B0, E0])
        args = (C, D, slip, fz, y_meas)

        if self.method == 'trust_region':
            be = self._tr_fit(_residuals_BE, be0, args, lb_be, ub_be)
        elif self.method == 'differential_evolution':
            be = self._de_fit(_cost_BE, args, lb_be, ub_be)
        elif self.method == 'genetic_algorithm':
            be = self._ga_fit(_cost_BE, args, lb_be, ub_be)
        elif self.method == 'ga_trust_region':
            be_ga = self._ga_fit(_cost_BE, args, lb_be, ub_be)
            be = self._tr_fit(_residuals_BE, be_ga, args, lb_be, ub_be)
        elif self.method == 'adaptive_de':
            be = self._jade_fit(_cost_BE, args, lb_be, ub_be)
        elif self.method == 'adaptive_de_trust_region':
            be_jade = self._jade_fit(_cost_BE, args, lb_be, ub_be)
            be = self._tr_fit(_residuals_BE, be_jade, args, lb_be, ub_be)
        elif self.method == 'bayesian_svi':
            be = self._svi_be(C, D, lb_be, ub_be, be0, slip, fz, y_meas)
        else:  # dual (DE → TR)
            be_global = self._de_fit(_cost_BE, args, lb_be, ub_be)
            be = self._tr_fit(_residuals_BE, be_global, args, lb_be, ub_be)

        B, E = be
        return np.array([B, C, D, E])

    # ──────────────────────────────────────────────────────────────────
    # Simultaneous identification (all 4 params at once)
    # ──────────────────────────────────────────────────────────────────

    def _simultaneous(self, slip, fz, y_meas, x0, label):
        x0 = np.array(x0, dtype=np.float64)
        channel = self._channel_from_label(label)
        C_lit = self.C_values[channel]

        if self.regularization == 'map':
            lb, ub = self._literature_bounds(channel)
            lam, sigma_C = self.map_reg['lambda_C'], self.map_reg['sigma_C']
            args = (slip, fz, y_meas, C_lit, lam, sigma_C)
            residuals_fn, cost_fn = _residuals_reg, _cost_reg
        else:
            lb, ub = self.lb, self.ub
            args = (slip, fz, y_meas)
            residuals_fn, cost_fn = _residuals, _cost

        if self.method == 'trust_region':
            return self._tr_fit(residuals_fn, x0, args, lb, ub)
        elif self.method == 'differential_evolution':
            return self._de_fit(cost_fn, args, lb, ub)
        elif self.method == 'genetic_algorithm':
            return self._ga_fit(cost_fn, args, lb, ub)
        elif self.method == 'ga_trust_region':
            ga_x = self._ga_fit(cost_fn, args, lb, ub)
            return self._tr_fit(residuals_fn, ga_x, args, lb, ub)
        elif self.method == 'adaptive_de':
            return self._jade_fit(cost_fn, args, lb, ub)
        elif self.method == 'adaptive_de_trust_region':
            jade_x = self._jade_fit(cost_fn, args, lb, ub)
            return self._tr_fit(residuals_fn, jade_x, args, lb, ub)
        elif self.method == 'bayesian_svi':
            return self._svi_all(C_lit, lb, ub, x0, slip, fz, y_meas)
        else:  # dual (DE → TR)
            de_x = self._de_fit(cost_fn, args, lb, ub)
            return self._tr_fit(residuals_fn, de_x, args, lb, ub)

    # ──────────────────────────────────────────────────────────────────
    # Bayesian SVI (Pyro) — correlated posterior + parameter uncertainty.
    # Lazily imports torch/pyro so the rest of the package (numpy/scipy
    # only) keeps working even if this optional, heavier dependency isn't
    # installed and this method is never selected.
    # ──────────────────────────────────────────────────────────────────

    def _svi_all(self, C_lit, lb, ub, x0, slip, fz, y_meas):
        import torch
        import pyro
        import pyro.distributions as dist
        from pyro.infer import SVI, Trace_ELBO
        from pyro.infer.autoguide import AutoMultivariateNormal

        slip_t = torch.as_tensor(slip, dtype=torch.float32)
        fz_t = torch.as_tensor(fz, dtype=torch.float32)
        y_t = torch.as_tensor(y_meas, dtype=torch.float32)

        B0, _, D0, E0 = x0
        ranges = np.asarray(ub, dtype=np.float64) - np.asarray(lb, dtype=np.float64)
        loc = {'B': float(B0), 'C': float(C_lit), 'D': float(D0), 'E': float(E0)}
        scale = {
            'B': max(ranges[0] / 4.0, 1e-3),
            'C': self.map_reg['sigma_C'],
            'D': max(ranges[2] / 4.0, 1e-3),
            'E': max(ranges[3] / 4.0, 1e-3),
        }
        names = ['B', 'C', 'D', 'E']

        def model():
            B = pyro.sample('B', dist.Normal(loc['B'], scale['B']))
            C = pyro.sample('C', dist.Normal(loc['C'], scale['C']))
            D = pyro.sample('D', dist.Normal(loc['D'], scale['D']))
            E = pyro.sample('E', dist.Normal(loc['E'], scale['E']))
            sigma_obs = pyro.sample('sigma_obs', dist.HalfNormal(50.0))
            x = B * slip_t
            mean = fz_t * D * torch.sin(C * torch.atan(x - E * (x - torch.atan(x))))
            with pyro.plate('data', slip_t.shape[0]):
                pyro.sample('obs', dist.Normal(mean, sigma_obs + 1e-6), obs=y_t)

        means, stds = self._run_svi(model, names)
        result = np.array([
            np.clip(means['B'], lb[0], ub[0]),
            np.clip(means['C'], lb[1], ub[1]),
            np.clip(means['D'], lb[2], ub[2]),
            np.clip(means['E'], lb[3], ub[3]),
        ])
        self._last_uncertainty = {f'{n}_std': stds[n] for n in names}
        return result

    def _svi_be(self, C, D, lb_be, ub_be, x0_be, slip, fz, y_meas):
        import torch
        import pyro
        import pyro.distributions as dist

        slip_t = torch.as_tensor(slip, dtype=torch.float32)
        fz_t = torch.as_tensor(fz, dtype=torch.float32)
        y_t = torch.as_tensor(y_meas, dtype=torch.float32)

        B0, E0 = x0_be
        ranges = np.asarray(ub_be, dtype=np.float64) - np.asarray(lb_be, dtype=np.float64)
        loc = {'B': float(B0), 'E': float(E0)}
        scale = {
            'B': max(ranges[0] / 4.0, 1e-3),
            'E': max(ranges[1] / 4.0, 1e-3),
        }
        names = ['B', 'E']

        def model():
            B = pyro.sample('B', dist.Normal(loc['B'], scale['B']))
            E = pyro.sample('E', dist.Normal(loc['E'], scale['E']))
            sigma_obs = pyro.sample('sigma_obs', dist.HalfNormal(50.0))
            x = B * slip_t
            mean = fz_t * D * torch.sin(C * torch.atan(x - E * (x - torch.atan(x))))
            with pyro.plate('data', slip_t.shape[0]):
                pyro.sample('obs', dist.Normal(mean, sigma_obs + 1e-6), obs=y_t)

        means, stds = self._run_svi(model, names)
        result = np.array([
            np.clip(means['B'], lb_be[0], ub_be[0]),
            np.clip(means['E'], lb_be[1], ub_be[1]),
        ])
        self._last_uncertainty = {f'{n}_std': stds[n] for n in names}
        return result

    def _run_svi(self, model, names, n_posterior_samples=300):
        """Train an AutoMultivariateNormal guide via SVI and summarise the
        posterior (mean/std per named site)."""
        import torch
        import pyro
        from pyro.infer import SVI, Trace_ELBO
        from pyro.infer.autoguide import AutoMultivariateNormal

        pyro.clear_param_store()
        if self.svi['seed'] is not None:
            pyro.set_rng_seed(self.svi['seed'])

        guide = AutoMultivariateNormal(model)
        optimizer = pyro.optim.Adam({'lr': self.svi['learning_rate']})
        svi = SVI(model, guide, optimizer, loss=Trace_ELBO())

        for _ in range(self.svi['num_steps']):
            svi.step()

        samples = {n: [] for n in names}
        with torch.no_grad():
            for _ in range(n_posterior_samples):
                sample = guide()
                for n in names:
                    samples[n].append(float(sample[n]))

        means = {n: float(np.mean(samples[n])) for n in names}
        stds = {n: float(np.std(samples[n])) for n in names}
        return means, stds
