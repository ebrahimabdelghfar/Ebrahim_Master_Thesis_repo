"""
Online Benchmarking Metrics for Vehicle Dynamics Estimation.

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
        Human-readable label for the signal being benchmarked
        (e.g. ``"v_y"`` or ``"omega"``).
    """

    def __init__(self, name: str = "signal"):
        self.name = name
        self.reset()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self):
        """Reset all running accumulators to their initial state."""
        self._n = 0                 # sample count

        # Accumulators for error statistics (Welford)
        self._sum_sq_err = 0.0      # Σ (e_i)²          → MSE
        self._sum_abs_err = 0.0     # Σ |e_i|           → MAE
        self._mean_err = 0.0        # running mean of e  (bias)
        self._M2_err = 0.0          # Welford M2 for error variance
        self._max_abs_err = 0.0     # max |e_i|

        # Accumulators for R² (need SS_tot of the true signal)
        self._mean_real = 0.0       # running mean of y
        self._M2_real = 0.0         # Welford M2 for variance of y
        self._sum_sq_res = 0.0      # Σ (y_i - ŷ_i)²   (= _sum_sq_err)

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

        # --- Squared / absolute error accumulators ---
        self._sum_sq_err += err * err
        self._sum_abs_err += abs(err)
        if abs(err) > self._max_abs_err:
            self._max_abs_err = abs(err)

        # --- Welford update for error mean & variance ---
        delta_err = err - self._mean_err
        self._mean_err += delta_err / n
        delta_err2 = err - self._mean_err
        self._M2_err += delta_err * delta_err2

        # --- Welford update for real-signal mean & variance (for R²) ---
        delta_real = real - self._mean_real
        self._mean_real += delta_real / n
        delta_real2 = real - self._mean_real
        self._M2_real += delta_real * delta_real2

        # SS_res is the same as sum of squared errors
        self._sum_sq_res = self._sum_sq_err

    def get_metrics(self) -> dict:
        """Return a dictionary of all benchmarking metrics.

        Returns
        -------
        dict
            Keys: ``rmse``, ``mae``, ``nrmse``, ``max_ae``, ``bias``,
            ``std_dev``, ``r_squared``, ``n_samples``.
            Values are ``float`` (or ``0.0`` / ``nan`` when undefined).
        """
        n = self._n
        if n == 0:
            return {
                'rmse': 0.0, 'mae': 0.0, 'nrmse': 0.0,
                'max_ae': 0.0, 'bias': 0.0, 'std_dev': 0.0,
                'r_squared': 0.0, 'n_samples': 0
            }

        mse = self._sum_sq_err / n
        rmse = math.sqrt(mse)
        mae = self._sum_abs_err / n
        bias = self._mean_err
        max_ae = self._max_abs_err

        # Error standard deviation (population)
        var_err = self._M2_err / n if n > 1 else 0.0
        std_dev = math.sqrt(var_err)

        # Normalized RMSE (by mean of true signal, avoid div-by-zero)
        mean_real = self._mean_real
        nrmse = rmse / abs(mean_real) if abs(mean_real) > 1e-12 else float('nan')

        # R² = 1 - SS_res / SS_tot
        ss_tot = self._M2_real  # Welford M2 = Σ(y_i - ȳ)²
        if ss_tot > 1e-12:
            r_squared = 1.0 - self._sum_sq_res / ss_tot
        else:
            r_squared = float('nan')

        return {
            'rmse': rmse,
            'mae': mae,
            'nrmse': nrmse,
            'max_ae': max_ae,
            'bias': bias,
            'std_dev': std_dev,
            'r_squared': r_squared,
            'n_samples': n,
        }

    def get_metrics_array(self) -> list:
        """Return metrics as a flat list in a fixed order.

        Order: ``[RMSE, MAE, NRMSE, MaxAE, Bias, StdDev, R²]``.
        Suitable for publishing as ``Float64MultiArray.data``.
        """
        m = self.get_metrics()
        return [
            m['rmse'], m['mae'], m['nrmse'],
            m['max_ae'], m['bias'], m['std_dev'],
            m['r_squared'],
        ]

    def get_summary_string(self) -> str:
        """Return a formatted multi-line summary for logging.

        The output is suitable for inclusion in an academic report or
        for printing to a ROS2 logger.
        """
        m = self.get_metrics()
        n = m['n_samples']
        if n == 0:
            return f"[{self.name}] No samples collected yet."

        r2_str = f"{m['r_squared']:.4f}" if not math.isnan(m['r_squared']) else "N/A"
        nrmse_str = f"{m['nrmse']:.4f}" if not math.isnan(m['nrmse']) else "N/A"

        lines = [
            f"=== Benchmarking: {self.name} (N={n}) ===",
            f"  RMSE           : {m['rmse']:.6f}",
            f"  MAE            : {m['mae']:.6f}",
            f"  NRMSE          : {nrmse_str}",
            f"  Max |error|    : {m['max_ae']:.6f}",
            f"  Bias (mean err): {m['bias']:.6f}",
            f"  Std Dev (err)  : {m['std_dev']:.6f}",
            f"  R^2            : {r2_str}",
        ]
        return "\n".join(lines)
