#!/usr/bin/env python3
"""
Pacejka Magic Formula coefficient identifier.

Provides two identification strategies:
  1. simultaneous  – fit all [B, C, D, E] at once (fast, but B/C/E are correlated)
  2. sequential    – physics-grounded: fix C, estimate D from peak, fit B & E
                     (breaks parameter degeneracy → unique, physically meaningful coefficients)

For ground-truth identification the *sequential* method is recommended: it
constrains C to physically meaningful values and identifies B, D, E with
proper physical interpretation. This avoids the well-known parameter
non-uniqueness problem of the Magic Formula where different (B, C, E)
combinations produce nearly identical curves.

Supported optimisers for the fitting step:
  - trust_region             (scipy.optimize.least_squares, TRF)
  - differential_evolution   (scipy.optimize.differential_evolution, global)
  - dual                     (DE → TRF, best accuracy)
  - genetic_algorithm        (custom real-coded GA, global)
  - ga_trust_region          (GA → TRF, hybrid global-local)
  - adaptive_de              (JADE — self-adaptive F/CR, global)
  - adaptive_de_trust_region (JADE → TRF, hybrid global-local)

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

            for i in range(self.NP):
                # --- Generate CR_i ~ N(mu_CR, 0.1), clipped to [0, 1] ---
                CR_i = np.clip(self.rng.normal(mu_CR, 0.1), 0.0, 1.0)

                # --- Generate F_i ~ Cauchy(mu_F, 0.1), truncated to (0, 1] ---
                F_i = self._rand_cauchy(mu_F, 0.1)

                # --- current-to-pbest/1 mutation ---
                # Select x_pbest: random from top-p fraction
                n_pbest = max(2, int(self.p * self.NP))
                pbest_indices = np.argsort(fitness)[:n_pbest]
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


def _residuals_BDE(bde_params, C, slip, fz, y_meas):
    """Residual with C fixed; B, D, E are free."""
    B, D, E = bde_params
    return pacejka_formula([B, C, D, E], slip, fz) - y_meas


def _cost_BDE(bde_params, C, slip, fz, y_meas):
    """Scalar cost with C fixed."""
    return float(np.sum(_residuals_BDE(bde_params, C, slip, fz, y_meas) ** 2))


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

def _estimate_D_from_peak(slip, fz, y_meas):
    """
    Estimate the peak factor D from data.

    D ≈ max|Y / Fz|  (the normalised peak force/torque).
    """
    fz_safe = np.where(np.abs(fz) > 1.0, fz, 1.0)
    y_norm = np.abs(y_meas / fz_safe)
    return float(np.percentile(y_norm, 99))  # 99th percentile (robust to outliers)


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


# ═══════════════════════════════════════════════════════════════════════════
# Main class
# ═══════════════════════════════════════════════════════════════════════════

# Default C values grounded in Pacejka literature
DEFAULT_C = {
    'fy': 1.30,    # lateral force shape factor (typical 1.1 – 1.8)
    'fx': 1.65,    # longitudinal force shape factor (typical 1.4 – 1.8)
    'mz': 2.40,    # self-aligning torque shape factor (typical 2.0 – 3.0)
}


class CoefficientIdentifier:
    """
    Identifies Pacejka [B, C, D, E] from measured (slip, Fz, force) data.

    Parameters
    ----------
    method : str
        Fitting strategy: 'trust_region', 'differential_evolution', 'dual',
        'genetic_algorithm', or 'ga_trust_region'.
    identification_mode : str
        'sequential' (recommended, fixes C) or 'simultaneous' (fits all 4).
    fixed_C : dict or None
        Override default C values, e.g. {'fy': 1.3, 'fx': 1.65, 'mz': 2.4}.
    lower_bounds / upper_bounds : list
        Bounds for [B, C, D, E] (simultaneous) or [B, D, E] (sequential).
    tr_params : dict or None
        Trust-Region hyperparameters: {max_nfev}.
    de_params : dict or None
        Differential Evolution hyperparameters: {maxiter, tol, seed, polish}.
    ga_params : dict or None
        Genetic Algorithm hyperparameters:
        {pop_size, n_generations, crossover_rate, mutation_rate,
         mutation_scale, elite_frac, tournament_size, seed}.
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
    ):
        allowed_methods = {
            'trust_region', 'differential_evolution', 'dual',
            'genetic_algorithm', 'ga_trust_region',
            'adaptive_de', 'adaptive_de_trust_region',
        }
        if method not in allowed_methods:
            raise ValueError(f"method must be one of {allowed_methods}")

        allowed_modes = {'sequential', 'simultaneous'}
        if identification_mode not in allowed_modes:
            raise ValueError(f"identification_mode must be one of {allowed_modes}")

        self.method = method
        self.id_mode = identification_mode

        # C values for sequential mode
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

        # Handle seed=-1 → None (non-deterministic)
        if self.de['seed'] == -1:
            self.de['seed'] = None
        if self.ga['seed'] == -1:
            self.ga['seed'] = None
        if self.jade['seed'] == -1:
            self.jade['seed'] = None

    # ──────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────

    def identify(self, slip, fz, y_meas, initial_guess=None, label=''):
        slip = np.asarray(slip, dtype=np.float64)
        fz = np.asarray(fz, dtype=np.float64)
        y_meas = np.asarray(y_meas, dtype=np.float64)

        if initial_guess is None:
            initial_guess = [10.0, 1.5, 1.0, 0.5]

        if self.id_mode == 'sequential':
            coeffs = self._sequential(slip, fz, y_meas, initial_guess, label)
        else:
            coeffs = self._simultaneous(slip, fz, y_meas, initial_guess)

        met = _metrics(coeffs, slip, fz, y_meas)
        coeffs_list = [round(float(c), 6) for c in coeffs]
        return coeffs_list, met

    def identify_fy(self, alpha, fz, fy, initial_guess=None):
        return self.identify(alpha, fz, fy, initial_guess, label='fy')

    def identify_fx(self, kappa, fz, fx, initial_guess=None):
        return self.identify(kappa, fz, fx, initial_guess, label='fx')

    def identify_mz(self, alpha, fz, mz, initial_guess=None):
        return self.identify(alpha, fz, mz, initial_guess, label='mz')

    # ──────────────────────────────────────────────────────────────────
    # Sequential identification (recommended)
    # ──────────────────────────────────────────────────────────────────

    def _sequential(self, slip, fz, y_meas, x0, label):
        """
        Physics-grounded sequential identification:
          1. Fix C to a literature-based value for the force channel
          2. Fit B, D, E with C fixed using the chosen optimiser
        This breaks the B-C-E degeneracy and produces unique, meaningful params.
        """
        # Determine C for this force channel
        label_lower = label.lower()
        if 'fx' in label_lower or 'longitudinal' in label_lower:
            C = self.C_values['fx']
        elif 'mz' in label_lower or 'aligning' in label_lower:
            C = self.C_values['mz']
        else:
            C = self.C_values['fy']

        # Bounds for [B, D, E] only
        lb_bde = np.array([self.lb[0], self.lb[2], self.lb[3]])
        ub_bde = np.array([self.ub[0], self.ub[2], self.ub[3]])

        # Initial guess for [B, D, E]
        B0, D0, E0 = x0[0], x0[2], x0[3]

        # Try to get a better D0 from data
        D_est = _estimate_D_from_peak(slip, fz, y_meas)
        if D_est is not None and 0.001 < D_est < 10.0:
            D0 = D_est

        bde0 = np.array([B0, D0, E0])

        if self.method == 'trust_region':
            bde = self._tr_bde(C, bde0, lb_bde, ub_bde, slip, fz, y_meas)
        elif self.method == 'differential_evolution':
            bde = self._de_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
        elif self.method == 'genetic_algorithm':
            bde = self._ga_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
        elif self.method == 'ga_trust_region':
            bde_ga = self._ga_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
            bde = self._tr_bde(C, bde_ga, lb_bde, ub_bde, slip, fz, y_meas)
        elif self.method == 'adaptive_de':
            bde = self._jade_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
        elif self.method == 'adaptive_de_trust_region':
            bde_jade = self._jade_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
            bde = self._tr_bde(C, bde_jade, lb_bde, ub_bde, slip, fz, y_meas)
        else:  # dual (DE → TR)
            bde_global = self._de_bde(C, lb_bde, ub_bde, slip, fz, y_meas)
            bde = self._tr_bde(C, bde_global, lb_bde, ub_bde, slip, fz, y_meas)

        B, D, E = bde
        return np.array([B, C, D, E])

    def _tr_bde(self, C, bde0, lb, ub, slip, fz, y):
        res = least_squares(
            _residuals_BDE, bde0, args=(C, slip, fz, y),
            bounds=(lb, ub), method='trf',
            max_nfev=self.tr['max_nfev'],
        )
        return res.x

    def _de_bde(self, C, lb, ub, slip, fz, y):
        bounds_list = list(zip(lb, ub))
        res = differential_evolution(
            _cost_BDE, bounds=bounds_list, args=(C, slip, fz, y),
            seed=self.de['seed'], maxiter=self.de['maxiter'],
            tol=self.de['tol'], polish=self.de['polish'],
        )
        return res.x

    def _ga_bde(self, C, lb, ub, slip, fz, y):
        bounds_list = list(zip(lb, ub))
        ga = GeneticAlgorithm(
            cost_fn=_cost_BDE,
            bounds=bounds_list,
            args=(C, slip, fz, y),
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

    def _jade_bde(self, C, lb, ub, slip, fz, y):
        bounds_list = list(zip(lb, ub))
        jade = JADE(
            cost_fn=_cost_BDE,
            bounds=bounds_list,
            args=(C, slip, fz, y),
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
    # Simultaneous identification (all 4 params at once)
    # ──────────────────────────────────────────────────────────────────

    def _simultaneous(self, slip, fz, y_meas, x0):
        x0 = np.array(x0, dtype=np.float64)
        if self.method == 'trust_region':
            return self._tr_all(x0, slip, fz, y_meas)
        elif self.method == 'differential_evolution':
            return self._de_all(slip, fz, y_meas)
        elif self.method == 'genetic_algorithm':
            return self._ga_all(slip, fz, y_meas)
        elif self.method == 'ga_trust_region':
            ga_x = self._ga_all(slip, fz, y_meas)
            return self._tr_all(ga_x, slip, fz, y_meas)
        elif self.method == 'adaptive_de':
            return self._jade_all(slip, fz, y_meas)
        elif self.method == 'adaptive_de_trust_region':
            jade_x = self._jade_all(slip, fz, y_meas)
            return self._tr_all(jade_x, slip, fz, y_meas)
        else:  # dual (DE → TR)
            de_x = self._de_all(slip, fz, y_meas)
            return self._tr_all(de_x, slip, fz, y_meas)

    def _tr_all(self, x0, slip, fz, y):
        res = least_squares(
            _residuals, x0, args=(slip, fz, y),
            bounds=(self.lb, self.ub), method='trf',
            max_nfev=self.tr['max_nfev'],
        )
        return res.x

    def _de_all(self, slip, fz, y):
        bounds_list = list(zip(self.lb, self.ub))
        res = differential_evolution(
            _cost, bounds=bounds_list, args=(slip, fz, y),
            seed=self.de['seed'], maxiter=self.de['maxiter'],
            tol=self.de['tol'], polish=self.de['polish'],
        )
        return res.x

    def _ga_all(self, slip, fz, y):
        bounds_list = list(zip(self.lb, self.ub))
        ga = GeneticAlgorithm(
            cost_fn=_cost,
            bounds=bounds_list,
            args=(slip, fz, y),
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

    def _jade_all(self, slip, fz, y):
        bounds_list = list(zip(self.lb, self.ub))
        jade = JADE(
            cost_fn=_cost,
            bounds=bounds_list,
            args=(slip, fz, y),
            pop_size=self.jade['pop_size'],
            n_generations=self.jade['n_generations'],
            p=self.jade['p'],
            c=self.jade['c'],
            archive_ratio=self.jade['archive_ratio'],
            seed=self.jade['seed'],
        )
        best_sol, _ = jade.run()
        return best_sol
