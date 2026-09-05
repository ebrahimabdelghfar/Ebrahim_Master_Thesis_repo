#!/usr/bin/env python3
"""Rollout-speed identifiability sweep (Fig. `identifiability_sweep`, Table `tab:sweep`).

Extends `On-Track-SysID/test/test_pacejka_identifiability.py`, which already
reproduces the mechanism in-process with no ROS and no torch. That module owns
the rollout (`_rollout`, i.e. `simulated_data_gen()` with the residual network
set to zero) and the bound test (`_on_bound`); this script only sweeps them.

The experiment: for each true peak factor D, roll a known tire out through the
synthetic steering sweep and fit it back with `solve_pacejka`, unregularised,
from the true coefficients as start point. Anything the fit loses is lost to
excitation, not to the solver or the prior.

Two axes of the bound are swept independently, because mu_reach has two of
them: rollout speed v at the inherited fixed delta_sweep, and delta_sweep at a
fixed v. If mu_reach is the governing variable then the collapse has to appear
at the same mu_reach on both.

Each cell is run once noise-free and once per seed with Gaussian noise added to
the rolled-out states before the fit. The noise-free fit is invariant to the
multi-start seed and to the jitter width (verified at 0.05 and 0.35), so the
seed axis carries no variance without it; the noise levels are the measured
one-step persistence RMSE of the two CARLA runs, sigma_vy = 0.009 m/s and
sigma_omega = 0.005 rad/s (`graphs/comparison/comparison_summary.csv`).

    python3 make_identifiability_sweep.py [--out-dir .]

Writes `identifiability_sweep.csv` (one row per axis x speed x delta_sweep x
D x seed) and
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
# Second axis: hold v and vary the sweep amplitude over the same mu_reach span.
# 0.54 rad is the shipped delta_max, so no cell asks for a sweep the pipeline
# would not run.
V_FIXED = 15.0
DELTA_SWEEPS = [0.03, 0.06, 0.10, 0.15, 0.25, 0.40, 0.54]
D_TRUE = [0.7, 1.05, 1.4, 1.8]
SEEDS = [0, 1, 2, 3, 4]
NUM_STARTS = 8             # the shipped value
SIGMA_VY = 0.009           # m/s      } measured one-step persistence RMSE of
SIGMA_OM = 0.005           # rad/s    } the two CARLA runs


def mu_reach(v, l_wb, delta_sweep=DELTA_SWEEP):
    """Eq. (7): the largest friction the rollout configuration can demand."""
    return v * v * delta_sweep / (G * l_wb)


def _fit(model, v, seed, noisy, delta_sweep=DELTA_SWEEP):
    vx, vy, om, delta = _rollout(model, v, delta_max=delta_sweep)
    # Realised lateral demand of the rollout, a_y = v_x*omega in quasi-steady
    # cornering. mu_reach is the Ackermann UPPER bound on this; the two part
    # company once the vehicle's own understeer bends the achieved curvature
    # away from delta/L, which is what the delta_sweep axis exposes.
    mu_real = float(np.max(np.abs(vx * om))) / G
    if noisy:
        rng = np.random.default_rng(10_000 + seed)
        vy = vy + rng.normal(0.0, SIGMA_VY, vy.shape)
        om = om + rng.normal(0.0, SIGMA_OM, om.shape)
    got_f, got_r = solve_pacejka(model, vx, vy, om, delta)
    return got_f, got_r, mu_real


def _cells():
    """(axis, v, delta_sweep) grid. The two axes share the v=15, ds=0.4 cell."""
    for v in SPEEDS:
        yield 'speed', v, DELTA_SWEEP
    for ds in DELTA_SWEEPS:
        yield 'delta', V_FIXED, ds


def run():
    rows = []
    for d in D_TRUE:
        tire = [6.3, 1.4, d, 0.0]
        for axis, v, ds in _cells():
            for noisy in (False, True):
                for seed in (SEEDS if noisy else [0]):
                    model = _vehicle(tire, tire, {
                        'pacejka_prior_weight': 0.0,     # isolate excitation
                        'pacejka_num_starts': NUM_STARTS,
                        'pacejka_seed': seed,
                    })
                    got_f, got_r, mu_real = _fit(model, v, seed, noisy, ds)
                    rows.append({
                        'axis': axis,
                        'D_true': d,
                        'v_rollout_mps': v,
                        'delta_sweep_rad': ds,
                        'mu_reach': round(mu_reach(v, model['l_wb'], ds), 4),
                        'mu_realised': round(mu_real, 4),
                        'noise': int(noisy),
                        'seed': seed,
                        'D_f': round(float(got_f[2]), 4),
                        'D_r': round(float(got_r[2]), 4),
                        'n_railed': len(_on_bound(got_f)) + len(_on_bound(got_r)),
                        'railed_f': '|'.join(_on_bound(got_f)),
                        'railed_r': '|'.join(_on_bound(got_r)),
                    })
                    print(f"[{axis}] D={d} v={v:>4} ds={ds:.2f} "
                          f"noise={int(noisy)} seed={seed}  "
                          f"D_f={rows[-1]['D_f']:.3f} D_r={rows[-1]['D_r']:.3f} "
                          f"railed={rows[-1]['n_railed']}/8", flush=True)
    return rows


def summarise(rows):
    """Mean and 95 % CI of D over noise realisations.

    The two axles of one rollout are not independent draws, so the interval is
    taken over the realisations only: each realisation contributes the mean of
    its own (D_f, D_r), and n is the number of realisations, not of fits.
    """
    out = {}
    for r in rows:
        if r['noise']:
            key = (r['axis'], r['D_true'], r['v_rollout_mps'],
                   r['delta_sweep_rad'])
            out.setdefault(key, []).append(r)
    table = []
    for (axis, d, v, ds), group in sorted(out.items()):
        vals = np.array([0.5 * (x['D_f'] + x['D_r']) for x in group])
        per_axle = {
            'D_f': np.array([x['D_f'] for x in group]),
            'D_r': np.array([x['D_r'] for x in group]),
        }
        n = len(vals)
        tcrit = 2.776 if n == 5 else 1.96  # t_{0.975, n-1} for the n = 5 grid
        half = tcrit * float(vals.std(ddof=1)) / math.sqrt(n) if n > 1 else 0.0
        row = {
            'axis': axis, 'D_true': d, 'v_rollout_mps': v,
            'delta_sweep_rad': ds, 'mu_reach': group[0]['mu_reach'],
            'mu_realised': round(float(np.mean(
                [x['mu_realised'] for x in group])), 4),
            'D_mean': round(float(vals.mean()), 4), 'D_ci95': round(half, 4),
            'railed_mean': round(float(np.mean([x['n_railed'] for x in group])), 2),
            'n_realisations': n,
        }
        for k, a in per_axle.items():
            row[k + '_mean'] = round(float(a.mean()), 4)
            row[k + '_ci95'] = round(
                tcrit * float(a.std(ddof=1)) / math.sqrt(n) if n > 1 else 0.0, 4)
        table.append(row)
    return table


def plot(table, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(7.0, 5.0), constrained_layout=True)
    (a_v, a_d), (a_rail, a_col) = ax
    for d in D_TRUE:
        spd = [t for t in table if t['D_true'] == d and t['axis'] == 'speed']
        dlt = [t for t in table if t['D_true'] == d and t['axis'] == 'delta']
        a_v.errorbar([t['v_rollout_mps'] for t in spd],
                     [t['D_mean'] for t in spd],
                     yerr=[t['D_ci95'] for t in spd], marker='o', capsize=2,
                     label=f'$D={d}$')
        a_v.axhline(d, linestyle=':', linewidth=0.8, color='0.6')
        a_d.errorbar([t['delta_sweep_rad'] for t in dlt],
                     [t['D_mean'] for t in dlt],
                     yerr=[t['D_ci95'] for t in dlt], marker='s', capsize=2,
                     label=f'$D={d}$')
        a_d.axhline(d, linestyle=':', linewidth=0.8, color='0.6')
        for sel, mark, lbl in ((spd, 'o', 'vary $v$'), (dlt, 's',
                                                        r'vary $\delta_{sweep}$')):
            a_rail.plot([t['mu_realised'] / d for t in sel],
                        [t['railed_mean'] for t in sel], marker=mark,
                        linestyle='-' if mark == 'o' else '--',
                        color=f'C{D_TRUE.index(d)}',
                        label=lbl if d == D_TRUE[0] else None)
            a_col.plot([t['mu_realised'] / d for t in sel],
                       [t['D_mean'] / d for t in sel], marker=mark,
                       linestyle='-' if mark == 'o' else '--',
                       color=f'C{D_TRUE.index(d)}',
                       label=lbl if d == D_TRUE[0] else None)

    a_v.set(xlabel='rollout speed [m/s]', ylabel='identified $D$',
            title=r'vary $v$ at $\delta_{sweep}=%.2f$ rad' % DELTA_SWEEP)
    a_d.set(xlabel=r'$\delta_{sweep}$ [rad]', ylabel='identified $D$',
            title=r'vary $\delta_{sweep}$ at $v=%.0f$ m/s' % V_FIXED)
    a_rail.set(xlabel=r'realised $\mu_{demand}/D$',
               ylabel='coefficients on a bound (of 8)',
               title='box-edge signature')
    a_col.set(xlabel=r'realised $\mu_{demand}/D$', ylabel=r'$\hat D/D$',
              title='both axes against the bound')
    a_col.axhline(1.0, linestyle=':', linewidth=0.8, color='0.4')
    for a in (a_rail, a_col):
        a.axvline(1.0, linestyle=':', linewidth=0.8, color='0.4')
        a.set_xscale('log')
    a_v.legend(fontsize=7)
    a_col.legend(fontsize=7)
    for a in ax.ravel():
        a.grid(alpha=0.3)
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
