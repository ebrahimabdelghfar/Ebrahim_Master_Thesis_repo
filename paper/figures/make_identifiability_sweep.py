#!/usr/bin/env python3
"""Rollout-speed identifiability sweep (Fig. `identifiability_sweep`, Table `tab:sweep`).

Extends `On-Track-SysID/test/test_pacejka_identifiability.py`, which already
reproduces the mechanism in-process with no ROS and no torch. That module owns
the rollout (`_rollout`, i.e. `simulated_data_gen()` with the residual network
set to zero) and the bound test (`_on_bound`); this script only sweeps them.

The experiment: for each rollout speed v and each true peak factor D, roll a
known tire out through the synthetic steering sweep and fit it back with
`solve_pacejka`, unregularised, from the true coefficients as start point.
Anything the fit loses is lost to excitation, not to the solver or the prior.

Each cell is run once noise-free and once per seed with Gaussian noise added to
the rolled-out states before the fit. The noise-free fit is invariant to the
multi-start seed and to the jitter width (verified at 0.05 and 0.35), so the
seed axis carries no variance without it; the noise levels are the measured
one-step persistence RMSE of the two CARLA runs, sigma_vy = 0.009 m/s and
sigma_omega = 0.005 rad/s (`graphs/comparison/comparison_summary.csv`).

    python3 make_identifiability_sweep.py [--out-dir .]

Writes `identifiability_sweep.csv` (one row per speed x D x seed) and
`identifiability_sweep.pdf`. Deterministic: the only stochastic element is the
multi-start jitter, seeded per row.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_TEST = os.path.join(_REPO, 'On-Track-SysID', 'test')
if _TEST not in sys.path:
    sys.path.insert(0, _TEST)

from test_pacejka_identifiability import _on_bound, _rollout, _vehicle  # noqa: E402
from helpers.solve_pacejka import solve_pacejka  # noqa: E402

G = 9.81
DELTA_SWEEP = 0.4          # rad; the inherited fixed sweep amplitude
SPEEDS = [4.0, 6.0, 8.0, 10.0, 13.0, 15.0, 20.0]
D_TRUE = [0.7, 1.05, 1.4, 1.8]
SEEDS = [0, 1, 2, 3, 4]
NUM_STARTS = 8             # the shipped value
SIGMA_VY = 0.009           # m/s      } measured one-step persistence RMSE of
SIGMA_OM = 0.005           # rad/s    } the two CARLA runs


def mu_reach(v, l_wb):
    """Eq. (7): the largest friction the rollout configuration can demand."""
    return v * v * DELTA_SWEEP / (G * l_wb)


def _fit(model, v, seed, noisy):
    vx, vy, om, delta = _rollout(model, v, delta_max=DELTA_SWEEP)
    if noisy:
        rng = np.random.default_rng(10_000 + seed)
        vy = vy + rng.normal(0.0, SIGMA_VY, vy.shape)
        om = om + rng.normal(0.0, SIGMA_OM, om.shape)
    got_f, got_r = solve_pacejka(model, vx, vy, om, delta)
    return got_f, got_r


def run():
    rows = []
    for d in D_TRUE:
        tire = [6.3, 1.4, d, 0.0]
        for v in SPEEDS:
            for noisy in (False, True):
                for seed in (SEEDS if noisy else [0]):
                    model = _vehicle(tire, tire, {
                        'pacejka_prior_weight': 0.0,     # isolate excitation
                        'pacejka_num_starts': NUM_STARTS,
                        'pacejka_seed': seed,
                    })
                    got_f, got_r = _fit(model, v, seed, noisy)
                    rows.append({
                        'D_true': d,
                        'v_rollout_mps': v,
                        'mu_reach': round(mu_reach(v, model['l_wb']), 4),
                        'noise': int(noisy),
                        'seed': seed,
                        'D_f': round(float(got_f[2]), 4),
                        'D_r': round(float(got_r[2]), 4),
                        'n_railed': len(_on_bound(got_f)) + len(_on_bound(got_r)),
                        'railed_f': '|'.join(_on_bound(got_f)),
                        'railed_r': '|'.join(_on_bound(got_r)),
                    })
                    print(f"D={d} v={v:>4} noise={int(noisy)} seed={seed}  "
                          f"D_f={rows[-1]['D_f']:.3f} D_r={rows[-1]['D_r']:.3f} "
                          f"railed={rows[-1]['n_railed']}/8", flush=True)
    return rows


def summarise(rows):
    """Mean and 95 % normal CI of the per-axle D over seeds, plus rail count."""
    out = {}
    for r in rows:
        if r['noise']:
            out.setdefault((r['D_true'], r['v_rollout_mps']), []).append(r)
    table = []
    for (d, v), group in sorted(out.items()):
        vals = np.array([x['D_f'] for x in group] + [x['D_r'] for x in group])
        n = len(vals)
        half = 1.96 * float(vals.std(ddof=1)) / math.sqrt(n) if n > 1 else 0.0
        table.append({
            'D_true': d, 'v_rollout_mps': v, 'mu_reach': group[0]['mu_reach'],
            'D_mean': round(float(vals.mean()), 4), 'D_ci95': round(half, 4),
            'railed_mean': round(float(np.mean([x['n_railed'] for x in group])), 2),
            'n_fits': n,
        })
    return table


def plot(table, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.8), constrained_layout=True)
    for d in D_TRUE:
        sel = [t for t in table if t['D_true'] == d]
        v = [t['v_rollout_mps'] for t in sel]
        axes[0].errorbar(v, [t['D_mean'] for t in sel],
                         yerr=[t['D_ci95'] for t in sel], marker='o',
                         capsize=2, label=f'$D={d}$')
        axes[0].axhline(d, linestyle=':', linewidth=0.8, color='0.6')
        axes[1].plot(v, [t['railed_mean'] for t in sel], marker='o',
                     label=f'$D={d}$')
    axes[0].set_xlabel('rollout speed [m/s]')
    axes[0].set_ylabel('identified $D$')
    axes[0].set_title('recovered peak factor (mean $\\pm$ 95 % CI)')
    axes[0].legend(fontsize=7)
    axes[1].set_xlabel('rollout speed [m/s]')
    axes[1].set_ylabel('coefficients on a bound (of 8)')
    axes[1].set_title('box-edge signature')
    axes[1].legend(fontsize=7)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.savefig(path)
    print('wrote', path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', default=os.path.dirname(os.path.abspath(__file__)))
    args = ap.parse_args()

    rows = run()
    raw = os.path.join(args.out_dir, 'identifiability_sweep.csv')
    with open(raw, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print('wrote', raw)

    table = summarise(rows)
    summ = os.path.join(args.out_dir, 'identifiability_sweep_summary.csv')
    with open(summ, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print('wrote', summ)

    plot(table, os.path.join(args.out_dir, 'identifiability_sweep.pdf'))


if __name__ == '__main__':
    main()
