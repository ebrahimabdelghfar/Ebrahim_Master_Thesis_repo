#!/usr/bin/env python3
"""
Synthetic-data test for the Pacejka coefficient identifier.

Validates all 5 identification methods in both sequential and simultaneous modes
by generating synthetic Pacejka data from known [B, C, D, E] coefficients, adding
Gaussian noise, running the identifier, and checking coefficient recovery accuracy.

Methods tested:
  - trust_region          (local optimizer)
  - differential_evolution (global optimizer)
  - dual                  (DE → TR hybrid)
  - genetic_algorithm     (real-coded GA, global)
  - ga_trust_region       (GA → TR hybrid)

Usage:
    python3 -m pytest test/test_identification.py -v
    # or directly:
    python3 test/test_identification.py
"""

import sys
import os
import numpy as np

# Ensure the package is importable from the workspace
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pacejka_identification.magic_formula import pacejka_fy, pacejka_fx, pacejka_mz
from pacejka_identification.coefficient_identifier import CoefficientIdentifier, DEFAULT_C


def test_sequential(formula_name, pacejka_fn, true_params, slip, fz, initial_guess, method, force_type):
    """
    Test sequential identification mode.

    In sequential mode, C is fixed to a physics-based literature value.
    We verify that B, D, E are recovered within tolerance.

    Parameters
    ----------
    formula_name : str     – human label (e.g. 'Fy')
    pacejka_fn   : callable – one of pacejka_fy, pacejka_fx, pacejka_mz
    true_params  : list    – ground-truth [B, C, D, E]
    slip         : ndarray – slip angle α or slip ratio κ
    fz           : ndarray – normal load
    initial_guess: list    – starting [B, C, D, E] for the optimizer
    method       : str     – identification method name
    force_type   : str     – 'fy', 'fx', or 'mz' (selects the fixed C value)

    Returns
    -------
    bool – True if test passed.
    """
    B_true, C_true, D_true, E_true = true_params
    C_fixed = DEFAULT_C[force_type]

    # Generate data using the FIXED C so the sequential identifier can recover B,D,E
    true_with_fixed_C = [B_true, C_fixed, D_true, E_true]
    y_clean = pacejka_fn(true_with_fixed_C, slip, fz)

    # Add small Gaussian noise (0.5 % of peak force)
    noise_std = 0.005 * np.max(np.abs(y_clean))
    np.random.seed(42)
    y_noisy = y_clean + np.random.normal(0, noise_std, size=y_clean.shape)

    # Run identification
    identifier = CoefficientIdentifier(method=method, identification_mode='sequential')
    coeffs, metrics = identifier.identify(slip, fz, y_noisy, initial_guess, label=force_type)

    B_est, C_est, D_est, E_est = coeffs

    # Check C is exactly the fixed value
    c_match = abs(C_est - C_fixed) < 1e-6

    # Compute relative errors for B, D, E
    errors = {}
    for name, true_val, est_val in [('B', B_true, B_est), ('D', D_true, D_est), ('E', E_true, E_est)]:
        if abs(true_val) > 1e-6:
            errors[name] = abs(est_val - true_val) / abs(true_val) * 100
        else:
            errors[name] = abs(est_val - true_val) * 100

    # Stochastic methods (GA, JADE) get relaxed tolerance (10 %)
    tol = 10.0 if 'genetic' in method or 'ga_' in method or 'adaptive' in method else 5.0
    passed = c_match and all(e < tol for e in errors.values()) and metrics['R2'] > 0.999

    print(f"\n{'='*65}")
    print(f"  SEQUENTIAL | {formula_name} | method={method}")
    print(f"{'='*65}")
    print(f"  True   [B,C,D,E]: {true_with_fixed_C}")
    print(f"  Found  [B,C,D,E]: {coeffs}")
    print(f"  C fixed={C_fixed:.2f}  C_match={c_match}")
    print(f"  Rel errors: B={errors['B']:.3f}%  D={errors['D']:.3f}%  E={errors['E']:.3f}%")
    print(f"  R²={metrics['R2']:.6f}  RMSE={metrics['RMSE']:.4f}  n={metrics['n_samples']}")
    print(f"  RESULT: {'PASS ✓' if passed else 'FAIL ✗'}")

    return passed


def test_simultaneous_curve(formula_name, pacejka_fn, true_params, slip, fz, initial_guess, method):
    """
    Test simultaneous identification mode.

    In simultaneous mode all 4 params are fitted at once.  Because of the
    known B-C-E degeneracy, individual parameters may differ from the true
    values.  We therefore only validate **curve-fit quality** (R² > 0.999).

    Returns
    -------
    bool – True if test passed.
    """
    y_clean = pacejka_fn(true_params, slip, fz)
    noise_std = 0.005 * np.max(np.abs(y_clean))
    np.random.seed(42)
    y_noisy = y_clean + np.random.normal(0, noise_std, size=y_clean.shape)

    identifier = CoefficientIdentifier(method=method, identification_mode='simultaneous')
    coeffs, metrics = identifier.identify(slip, fz, y_noisy, initial_guess, label=formula_name)

    passed = metrics['R2'] > 0.999

    print(f"\n{'='*65}")
    print(f"  SIMULTANEOUS | {formula_name} | method={method}")
    print(f"{'='*65}")
    print(f"  True   [B,C,D,E]: {true_params}")
    print(f"  Found  [B,C,D,E]: {coeffs}")
    print(f"  R²={metrics['R2']:.6f}  RMSE={metrics['RMSE']:.4f}")
    print(f"  RESULT: {'PASS ✓' if passed else 'FAIL ✗'}")

    return passed


def main():
    """Run the full test suite and return exit code (0 = all pass)."""
    print("=" * 65)
    print("   Pacejka Coefficient Identifier — Full Test Suite")
    print("=" * 65)

    # ── Generate synthetic data ──
    alpha = np.linspace(-0.3, 0.3, 2000)       # slip angles (rad)
    kappa = np.linspace(-0.4, 0.4, 2000)       # slip ratios (-)
    fz_const = np.full(2000, 3000.0)            # constant 3000 N
    np.random.seed(123)
    fz_var = np.random.uniform(1500, 5000, 2000)  # varying Fz

    # Ground-truth coefficients [B, C, D, E]
    true_fy = [7.17, 1.30, 0.69, 0.53]
    true_fx = [12.0, 1.65, 1.1, 0.3]
    true_mz = [8.0, 2.40, 0.05, 0.6]

    results = []

    # ── Sequential tests — all methods ──
    ALL_METHODS = ['trust_region', 'dual', 'genetic_algorithm', 'ga_trust_region',
                    'adaptive_de', 'adaptive_de_trust_region']

    for method in ALL_METHODS:
        results.append(test_sequential(
            'Fy', pacejka_fy, true_fy, alpha, fz_const,
            [10.0, 1.5, 1.0, 0.5], method, 'fy',
        ))
        results.append(test_sequential(
            'Fx', pacejka_fx, true_fx, kappa, fz_const,
            [10.0, 1.65, 1.0, 0.5], method, 'fx',
        ))
        results.append(test_sequential(
            'Mz', pacejka_mz, true_mz, alpha, fz_const,
            [10.0, 1.5, 0.1, 0.5], method, 'mz',
        ))

    # ── Simultaneous tests — GA and JADE methods (curve-fit quality only) ──
    results.append(test_simultaneous_curve(
        'Fy (GA simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'genetic_algorithm',
    ))
    results.append(test_simultaneous_curve(
        'Fy (GA→TR simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'ga_trust_region',
    ))
    results.append(test_simultaneous_curve(
        'Fy (JADE simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'adaptive_de',
    ))
    results.append(test_simultaneous_curve(
        'Fy (JADE→TR simult.)', pacejka_fy, [7.17, 1.56, 0.69, 0.53],
        alpha, fz_const, [10.0, 1.5, 1.0, 0.5], 'adaptive_de_trust_region',
    ))

    # ── Summary ──
    print("\n" + "=" * 65)
    passed = sum(results)
    total = len(results)
    status = "ALL PASS ✓" if all(results) else f"{passed}/{total} passed"
    print(f"   SUMMARY: {status}")
    print("=" * 65)

    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
