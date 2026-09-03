#!/usr/bin/env python3
"""
Synthetic-data test for the Pacejka coefficient identifier.

Validates all identification methods in both sequential and simultaneous
modes by generating synthetic Pacejka data from known [B, C, D, E]
coefficients, adding Gaussian noise, running the identifier, and checking
coefficient recovery accuracy. Also covers the simultaneous-mode MAP
regularization and the `bayesian_svi` method (both modes).

Methods tested (sequential):
  - trust_region            (local optimizer)
  - differential_evolution  (global optimizer)
  - dual                    (DE -> TR hybrid)
  - genetic_algorithm       (real-coded GA, global)
  - ga_trust_region         (GA -> TR hybrid)
  - adaptive_de             (JADE, global)
  - adaptive_de_trust_region (JADE -> TR hybrid)
  - bayesian_svi            (Pyro SVI, correlated posterior)

Usage:
    python3 test/test_identification.py
    python3 -m pytest test/test_identification.py -v
"""

import sys
import os
import numpy as np

# Ensure the package is importable from the workspace
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pacejka_identification.magic_formula import pacejka_fy, pacejka_fx, pacejka_mz
from pacejka_identification.coefficient_identifier import CoefficientIdentifier


def run_sequential_case(formula_name, pacejka_fn, true_params, slip, fz, initial_guess,
                         method, force_type, ci_kwargs=None):
    """
    Run sequential identification mode on synthetic data.

    In sequential mode only D is fixed — to the (bounds-clipped) peak estimate
    from the data — while B, C and E are fitted. Fixing the peak pins down D
    (= mu) but leaves the classic B-C-E degeneracy: several shape triples give
    near-identical curves, so those are checked as a *curve* rather than
    parameter-by-parameter (the individual errors are printed for reference).

    Parameters
    ----------
    formula_name : str     - human label (e.g. 'Fy')
    pacejka_fn   : callable - one of pacejka_fy, pacejka_fx, pacejka_mz
    true_params  : list    - ground-truth [B, C, D, E]
    slip         : ndarray - slip angle alpha or slip ratio kappa
    fz           : ndarray - normal load
    initial_guess: list    - starting [B, C, D, E] for the optimizer
    method       : str     - identification method name
    force_type   : str     - 'fy', 'fx', or 'mz' (channel label)
    ci_kwargs    : dict or None - extra CoefficientIdentifier constructor args

    Returns
    -------
    bool - True if test passed.
    """
    B_true, C_true, D_true, E_true = true_params
    y_clean = pacejka_fn(true_params, slip, fz)

    # Add small Gaussian noise (0.5 % of peak force)
    noise_std = 0.005 * np.max(np.abs(y_clean))
    np.random.seed(42)
    y_noisy = y_clean + np.random.normal(0, noise_std, size=y_clean.shape)

    # Run identification
    identifier = CoefficientIdentifier(method=method, identification_mode='sequential',
                                        **(ci_kwargs or {}))
    coeffs, metrics = identifier.identify(slip, fz, y_noisy, initial_guess, label=force_type)

    B_est, C_est, D_est, E_est = coeffs

    # Ground-truth mu: peak of the true curve, which the data peak estimates.
    mu_true = np.max(np.abs(y_clean / fz))

    errors = {}
    for name, true_val, est_val in [('B', B_true, B_est), ('C', C_true, C_est),
                                    ('D', D_true, D_est), ('E', E_true, E_est),
                                    ('mu', mu_true, metrics['mu_data'])]:
        if abs(true_val) > 1e-6:
            errors[name] = abs(est_val - true_val) / abs(true_val) * 100
        else:
            errors[name] = abs(est_val - true_val) * 100

    # Curve fidelity against the noise-free truth (degeneracy-proof check)
    curve_err = np.max(np.abs(pacejka_fn(coeffs, slip, fz) - y_clean)) / np.max(np.abs(y_clean)) * 100

    passed = (errors['D'] < 2.0 and errors['mu'] < 2.0
              and curve_err < 2.0 and metrics['R2'] > 0.999)

    print(f"\n{'='*65}")
    print(f"  SEQUENTIAL | {formula_name} | method={method}")
    print(f"{'='*65}")
    print(f"  True   [B,C,D,E]: {true_params}")
    print(f"  Found  [B,C,D,E]: {coeffs}")
    print(f"  Rel errors: B={errors['B']:.3f}%  C={errors['C']:.3f}%  "
          f"D={errors['D']:.3f}%  E={errors['E']:.3f}%  mu={errors['mu']:.3f}%")
    print(f"  Max curve deviation vs truth: {curve_err:.3f}%")
    print(f"  mu: true={mu_true:.4f}  data={metrics['mu_data']:.4f}  model={metrics['mu_model']:.4f}")
    print(f"  R²={metrics['R2']:.6f}  RMSE={metrics['RMSE']:.4f}  n={metrics['n_samples']}")
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")

    return passed


def run_simultaneous_case(formula_name, pacejka_fn, true_params, slip, fz, initial_guess,
                           method, ci_kwargs=None):
    """
    Run simultaneous identification mode on synthetic data.

    In simultaneous mode all 4 params are fitted at once. Because of the
    known B-C-E degeneracy, individual parameters may differ from the true
    values. We therefore only validate **curve-fit quality** (R2 > 0.999).

    Returns
    -------
    bool - True if test passed.
    """
    y_clean = pacejka_fn(true_params, slip, fz)
    noise_std = 0.005 * np.max(np.abs(y_clean))
    np.random.seed(42)
    y_noisy = y_clean + np.random.normal(0, noise_std, size=y_clean.shape)

    identifier = CoefficientIdentifier(method=method, identification_mode='simultaneous',
                                        **(ci_kwargs or {}))
    coeffs, metrics = identifier.identify(slip, fz, y_noisy, initial_guess, label=formula_name)

    passed = metrics['R2'] > 0.999

    print(f"\n{'='*65}")
    print(f"  SIMULTANEOUS | {formula_name} | method={method} | kwargs={ci_kwargs}")
    print(f"{'='*65}")
    print(f"  True   [B,C,D,E]: {true_params}")
    print(f"  Found  [B,C,D,E]: {coeffs}")
    print(f"  R²={metrics['R2']:.6f}  RMSE={metrics['RMSE']:.4f}")
    print(f"  RESULT: {'PASS' if passed else 'FAIL'}")

    return passed


def main():
    """Run the full test suite and return exit code (0 = all pass)."""
    print("=" * 65)
    print("   Pacejka Coefficient Identifier - Full Test Suite")
    print("=" * 65)

    # -- Generate synthetic data --
    alpha = np.linspace(-0.3, 0.3, 2000)       # slip angles (rad)
    kappa = np.linspace(-0.4, 0.4, 2000)       # slip ratios (-)
    fz_const = np.full(2000, 3000.0)            # constant 3000 N
    np.random.seed(123)
    fz_var = np.random.uniform(1500, 5000, 2000)  # varying Fz

    # Ground-truth coefficients [B, C, D, E]
    true_fy = [12.17, 1.30, 0.69, 0.53]
    true_fx = [12.0, 1.65, 1.1, 0.3]
    true_mz = [8.0, 2.40, 0.05, 0.6]

    results = []

    # -- Sequential tests -- all methods (including bayesian_svi) --
    ALL_METHODS = ['trust_region', 'differential_evolution', 'dual', 'genetic_algorithm',
                   'ga_trust_region', 'adaptive_de', 'adaptive_de_trust_region']
    # bayesian_svi is stochastic/slower (Pyro SVI) — run with fewer steps for the test suite
    SVI_KWARGS = {'svi_params': {'num_steps': 500}}

    for method in ALL_METHODS:
        results.append(run_sequential_case(
            'Fy', pacejka_fy, true_fy, alpha, fz_const,
            [10.0, 1.5, 1.0, 0.5], method, 'fy',
        ))
        results.append(run_sequential_case(
            'Fx', pacejka_fx, true_fx, kappa, fz_const,
            [10.0, 1.65, 1.0, 0.5], method, 'fx',
        ))
        results.append(run_sequential_case(
            'Mz', pacejka_mz, true_mz, alpha, fz_const,
            [10.0, 1.5, 0.1, 0.5], method, 'mz',
        ))

    results.append(run_sequential_case(
        'Fy', pacejka_fy, true_fy, alpha, fz_const,
        [10.0, 1.5, 1.0, 0.5], 'bayesian_svi', 'fy', ci_kwargs=SVI_KWARGS,
    ))

    # -- Simultaneous tests -- GA and JADE methods (curve-fit quality only) --
    results.append(run_simultaneous_case(
        'Fy (GA simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'genetic_algorithm',
    ))
    results.append(run_simultaneous_case(
        'Fy (GA->TR simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'ga_trust_region',
    ))
    results.append(run_simultaneous_case(
        'Fy (JADE simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'adaptive_de',
    ))
    results.append(run_simultaneous_case(
        'Fy (JADE->TR simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'adaptive_de_trust_region',
    ))

    # -- Simultaneous MAP regularization (literature bounds + Gaussian prior on C) --
    results.append(run_simultaneous_case(
        'Fy (dual, MAP reg.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'dual',
        ci_kwargs={'regularization': 'map'},
    ))

    # -- Simultaneous bayesian_svi (correlated posterior) --
    # The 4-parameter joint posterior converges more slowly than the
    # 3-parameter (B,C,E) sequential case, so it gets more SVI steps here.
    results.append(run_simultaneous_case(
        'Fy (bayesian_svi simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'bayesian_svi',
        ci_kwargs={'svi_params': {'num_steps': 3000}},
    ))

    # -- Summary --
    print("\n" + "=" * 65)
    passed = sum(results)
    total = len(results)
    status = "ALL PASS" if all(results) else f"{passed}/{total} passed"
    print(f"   SUMMARY: {status}")
    print("=" * 65)

    return 0 if all(results) else 1


def test_all_methods():
    """Real pytest entry point (so `pytest -v` actually runs the suite)."""
    assert main() == 0


if __name__ == '__main__':
    sys.exit(main())
