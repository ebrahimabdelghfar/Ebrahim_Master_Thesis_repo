"""
Online Benchmarking Metrics for Estimation Quality Assessment.

Provides incremental (O(1) per sample) computation of standard academic
benchmarking metrics using Welford's online algorithm for numerically
stable running statistics.

Metrics computed:
    - RMSE  : Root Mean Squared Error
    - MAE   : Mean Absolute Error
    - NRMSE : Normalized RMSE (by mean of true values)
    - MaxAE : Maximum Absolute Error
    - Bias  : Mean signed error (systematic bias)
    - StdDev: Standard deviation of the error
    - R²    : Coefficient of determination

Reference:
    Welford, B. P. (1962). "Note on a method for calculating corrected
    sums of squares and products." Technometrics, 4(3), 419-420.
"""

import math


class OnlineBenchmark:
    """Incrementally computes academic benchmarking metrics.

    Uses Welford's algorithm for numerically stable one-pass computation
    of variance-related statistics.  Each call to :meth:`update` is O(1).

    Attributes
    ----------
    name : str
        Human-readable label for the signal being benchmarked.
    """

    def __init__(self, name: str = "signal"):
        self.name = name
        self.reset()

    def reset(self):
        """Reset all running accumulators to their initial state."""
        self._n = 0

        # Error accumulators (Welford)
        self._sum_sq_err = 0.0
        self._sum_abs_err = 0.0
        self._mean_err = 0.0
        self._M2_err = 0.0
        self._max_abs_err = 0.0

        # True-signal accumulators for R²
        self._mean_real = 0.0
        self._M2_real = 0.0

    def update(self, real: float, predicted: float):
        """Feed one (real, predicted) sample pair.

        Parameters
        ----------
        real : float
            Ground-truth / sensor measurement.
        predicted : float
            Model prediction for this timestep.
        """
        self._n += 1
        n = self._n
        err = real - predicted

        self._sum_sq_err += err * err
        self._sum_abs_err += abs(err)
        if abs(err) > self._max_abs_err:
            self._max_abs_err = abs(err)

        # Welford update for error mean & variance
        delta_err = err - self._mean_err
        self._mean_err += delta_err / n
        delta_err2 = err - self._mean_err
        self._M2_err += delta_err * delta_err2

        # Welford update for real-signal mean & variance (for R²)
        delta_real = real - self._mean_real
        self._mean_real += delta_real / n
        delta_real2 = real - self._mean_real
        self._M2_real += delta_real * delta_real2

    def metrics(self) -> dict:
        """Return a dictionary of all benchmarking metrics.

        Returns
        -------
        dict
            Keys: rmse, mae, nrmse, max_ae, bias, std_dev, r_squared, n_samples.
        """
        n = self._n
        if n == 0:
            return {
                'rmse': 0.0, 'mae': 0.0, 'nrmse': float('nan'),
                'max_ae': 0.0, 'bias': 0.0, 'std_dev': 0.0,
                'r_squared': float('nan'), 'n_samples': 0,
            }

        mse = self._sum_sq_err / n
        rmse = math.sqrt(mse)
        mae = self._sum_abs_err / n
        bias = self._mean_err
        var_err = self._M2_err / n if n > 1 else 0.0
        std_dev = math.sqrt(var_err)
        nrmse = rmse / abs(self._mean_real) if abs(self._mean_real) > 1e-12 else float('nan')

        ss_tot = self._M2_real
        r_squared = 1.0 - (self._sum_sq_err / ss_tot) if ss_tot > 1e-12 else float('nan')

        return {
            'rmse': rmse,
            'mae': mae,
            'nrmse': nrmse,
            'max_ae': self._max_abs_err,
            'bias': bias,
            'std_dev': std_dev,
            'r_squared': r_squared,
            'n_samples': n,
        }

    def metrics_array(self) -> list:
        """Return metrics as a flat list: [RMSE, MAE, NRMSE, MaxAE, Bias, StdDev, R²]."""
        m = self.metrics()
        return [
            m['rmse'], m['mae'], m['nrmse'],
            m['max_ae'], m['bias'], m['std_dev'],
            m['r_squared'],
        ]

    def summary(self) -> str:
        """Return a formatted single-line summary for logging."""
        m = self.metrics()
        if m['n_samples'] == 0:
            return f'[{self.name}] no samples'

        r2 = f"{m['r_squared']:.4f}" if not math.isnan(m['r_squared']) else 'N/A'
        nrmse = f"{m['nrmse']:.4f}" if not math.isnan(m['nrmse']) else 'N/A'

        return (
            f"[{self.name}] N={m['n_samples']} RMSE={m['rmse']:.6f} MAE={m['mae']:.6f} "
            f"NRMSE={nrmse} MaxAE={m['max_ae']:.6f} Bias={m['bias']:.6f} "
            f"Std={m['std_dev']:.6f} R2={r2}"
        )
