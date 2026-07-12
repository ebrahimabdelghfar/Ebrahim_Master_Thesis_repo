import math


class HistoryBuffer:
    """Bounded-memory (ground truth, estimate) time-history for plotting.

    Once more than 2*max_points samples have been seen, the buffer halves
    itself (keeping every other point) and doubles its sampling stride, so
    memory stays bounded on long-running nodes while still spanning the
    full run (logarithmic decimation, same idea as RRDtool).
    """

    def __init__(self, max_points: int = 5000):
        self.max_points = max(100, int(max_points))
        self._stride = 1
        self._count = 0
        self.stamps = []
        self.gt = []
        self.est = []

    def add(self, stamp: float, gt: float, est: float):
        self._count += 1
        if (self._count - 1) % self._stride != 0:
            return
        self.stamps.append(stamp)
        self.gt.append(gt)
        self.est.append(est)
        if len(self.stamps) > 2 * self.max_points:
            self.stamps = self.stamps[::2]
            self.gt = self.gt[::2]
            self.est = self.est[::2]
            self._stride *= 2


class OnlineBenchmark:
    def __init__(self, name: str = 'signal'):
        self.name = name
        self.reset()

    def reset(self):
        self._n = 0
        self._sum_sq_err = 0.0
        self._sum_abs_err = 0.0
        self._mean_err = 0.0
        self._m2_err = 0.0
        self._max_abs_err = 0.0
        self._mean_real = 0.0
        self._m2_real = 0.0

    def update(self, real: float, predicted: float):
        self._n += 1
        n = self._n
        err = real - predicted

        self._sum_sq_err += err * err
        self._sum_abs_err += abs(err)
        self._max_abs_err = max(self._max_abs_err, abs(err))

        delta_err = err - self._mean_err
        self._mean_err += delta_err / n
        delta_err2 = err - self._mean_err
        self._m2_err += delta_err * delta_err2

        delta_real = real - self._mean_real
        self._mean_real += delta_real / n
        delta_real2 = real - self._mean_real
        self._m2_real += delta_real * delta_real2

    def metrics(self) -> dict:
        if self._n == 0:
            return {
                'rmse': 0.0,
                'mae': 0.0,
                'nrmse': float('nan'),
                'max_ae': 0.0,
                'bias': 0.0,
                'std_dev': 0.0,
                'r_squared': float('nan'),
                'n_samples': 0,
            }

        n = self._n
        mse = self._sum_sq_err / n
        rmse = math.sqrt(mse)
        mae = self._sum_abs_err / n
        bias = self._mean_err
        var_err = self._m2_err / n if n > 1 else 0.0
        std_dev = math.sqrt(var_err)
        nrmse = rmse / abs(self._mean_real) if abs(self._mean_real) > 1e-12 else float('nan')
        ss_tot = self._m2_real
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

    def summary(self) -> str:
        m = self.metrics()
        if m['n_samples'] == 0:
            return f'[{self.name}] no samples'

        r2 = f"{m['r_squared']:.4f}" if not math.isnan(m['r_squared']) else 'N/A'
        nrmse = f"{m['nrmse']:.4f}" if not math.isnan(m['nrmse']) else 'N/A'

        return (
            f"[{self.name}] N={m['n_samples']} RMSE={m['rmse']:.3f} MAE={m['mae']:.3f} "
            f"NRMSE={nrmse} MaxAE={m['max_ae']:.3f} Bias={m['bias']:.3f} "
            f"Std={m['std_dev']:.3f} R2={r2}"
        )
