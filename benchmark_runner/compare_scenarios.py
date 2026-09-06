#!/usr/bin/env python3
"""Build the cross-scenario comparison directory from a completed sweep.

Reads the per-plot CSVs that `tire_force_benchmark` and
`adaptive_controller_benchmark` write beside each figure, so nothing here
re-derives a metric or a tire curve - the comparison and the per-scenario
figures cannot disagree.

    make compare_benchmark_scenarios

Writes `<graphs_root>/comparison/`: one `comparison_summary.csv` with a row per
scenario, plus a PNG and a same-named CSV per comparison figure.
"""
import argparse
import csv
import sys
from collections import OrderedDict
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))

from run_status import read_failure_reason  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent

FAILURE_COLOR = '#b00020'

AXLE_FORCE_SIGNALS = ('front_sum_fy', 'rear_sum_fy')
STATE_SIGNALS = ('v_y', 'omega')
MU_SIGNALS = ('front_mu', 'rear_mu')
STATE_UNITS = {'v_y': 'm/s', 'omega': 'rad/s'}


# ---------------- CSV loading ----------------

def _read_rows(path):
    """Rows of a per-plot CSV as dicts, or [] when the file is absent.

    A missing file is normal: figures skip themselves when they have no data
    (`lap_times.png` when no lap completed, the Pacejka figures before the first
    identification), and their CSVs follow the same rule.
    """
    path = Path(path)
    if not path.is_file():
        return []
    with path.open(newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _floats(rows, column):
    out = []
    for row in rows:
        try:
            out.append(float(row[column]))
        except (TypeError, ValueError):
            continue
    return out


def _group(rows, key_column, value_column):
    grouped = OrderedDict()
    for row in rows:
        try:
            value = float(row[value_column])
        except (TypeError, ValueError):
            continue
        grouped.setdefault(row[key_column], []).append(value)
    return grouped


class ScenarioData:
    """Everything one scenario contributes, loaded once."""

    def __init__(self, name, graphs_root):
        self.name = name
        self.ident_dir = graphs_root / 'identification' / name
        self.control_dir = graphs_root / 'control' / name

        # Either directory carries the sidecar; the identification one is written
        # first, so prefer it and fall back in case only one survived.
        self.failure_reason = (read_failure_reason(self.ident_dir)
                               or read_failure_reason(self.control_dir))

        self.control_metrics = self._control_metrics()
        self.ident_metrics = self._ident_metrics()
        self.lap_times = _floats(_read_rows(self.control_dir / 'lap_times.csv'), 'lap_time_s')
        self.abs_error = _read_rows(
            self.control_dir / 'tracking_error_boxplot_by_controller.csv')
        self.tracking_series = _read_rows(
            self.control_dir / 'tracking_error_timeseries.csv')
        self.force_errors = _group(
            _read_rows(self.ident_dir / 'tire_forces_error_hist.csv'),
            'signal', 'error_gt_minus_est')
        self.state_errors = _group(
            _read_rows(self.ident_dir / 'vehicle_states_error_hist.csv'),
            'signal', 'error_gt_minus_est')
        self.force_series = _read_rows(self.ident_dir / 'tire_forces_timeseries.csv')
        self.state_series = _read_rows(self.ident_dir / 'vehicle_states_timeseries.csv')
        self.mu_series = _read_rows(self.ident_dir / 'friction_mu_timeseries.csv')
        self.pacejka = _read_rows(self.ident_dir / 'pacejka_identified_vs_nominal.csv')

    def exists(self):
        return bool(self.control_metrics or self.ident_metrics)

    def _control_metrics(self):
        """metrics_summary_table.csv is tidy (section, key, metric, value)."""
        out = {}
        for row in _read_rows(self.control_dir / 'metrics_summary_table.csv'):
            out[(row['section'], row['key'], row['metric'])] = row['value']
        return out

    def _ident_metrics(self):
        """metrics_summary.csv is one row per signal, keyed by signal_key."""
        return {row['signal_key']: row
                for row in _read_rows(self.ident_dir / 'metrics_summary.csv')}

    # convenience accessors

    def control(self, section, key, metric):
        value = self.control_metrics.get((section, key, metric), '')
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def ident(self, signal_key, column):
        row = self.ident_metrics.get(signal_key)
        if row is None:
            return None
        try:
            return float(row[column])
        except (TypeError, ValueError, KeyError):
            return None

    def pacejka_coefficients(self, axle):
        for row in self.pacejka:
            if row['axle'] == axle:
                try:
                    return [float(row[c]) for c in ('B', 'C', 'D', 'E')]
                except (TypeError, ValueError):
                    return None
        return None

    def pacejka_curve(self, axle):
        alpha, identified, nominal = [], [], []
        for row in self.pacejka:
            if row['axle'] != axle:
                continue
            try:
                alpha.append(float(row['alpha_rad']))
                identified.append(float(row['fy_identified_N']))
                nominal.append(float(row['fy_nominal_N']))
            except (TypeError, ValueError):
                continue
        return alpha, identified, nominal

    def series(self, rows, signal):
        t, gt, est = [], [], []
        for row in rows:
            if row.get('signal') != signal:
                continue
            try:
                t.append(float(row['t_s']))
                gt.append(float(row['ground_truth']))
                est.append(float(row['estimate']))
            except (TypeError, ValueError):
                continue
        return t, gt, est


# ---------------- output helpers ----------------

class Comparison:

    def __init__(self, out_dir, plt, incomplete=()):
        self.out_dir = Path(out_dir)
        self.plt = plt
        self.written = []
        # [(scenario, reason)] for the runs that did not finish. Every figure
        # carries the caveat, because any of them can mix a short run in with
        # complete ones and nothing else on the axes would say so.
        self.incomplete = list(incomplete)

    def _banner(self, fig):
        if not self.incomplete:
            return
        detail = '; '.join(f'{name} ({reason})' for name, reason in self.incomplete)
        fig.text(0.5, 0.012, f'Incomplete run(s) included: {detail}',
                 ha='center', va='bottom', fontsize=6, color=FAILURE_COLOR, wrap=True)

    def save(self, fig, basename, header, rows):
        self._banner(fig)
        # Reserve the bottom strip when there is a banner: tight_layout does not
        # account for figure-level text and would lay the axes over it.
        fig.tight_layout(rect=(0, 0.05, 1, 1) if self.incomplete else None)
        # PDF is what the paper includes (IEEE wants >= 300 dpi and these are
        # line plots, so vector is both smaller and exact); the PNG stays for
        # quick viewing outside LaTeX.
        fig.savefig(self.out_dir / f'{basename}.pdf')
        fig.savefig(self.out_dir / f'{basename}.png', dpi=150)
        self.plt.close(fig)
        with (self.out_dir / f'{basename}.csv').open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
        self.written.append(basename)

    def grouped_bars(self, basename, title, ylabel, series, categories):
        """One bar group per category, one bar per scenario.

        `series` is {scenario: {category: value}}; a missing or None value is
        left out rather than drawn as zero, which would read as a measurement.
        """
        import numpy as np
        fig, ax = self.plt.subplots(
            figsize=(max(6.0, 1.4 * len(categories) + 0.8 * len(series)), 4))
        n = max(1, len(series))
        width = 0.8 / n
        rows = []
        for idx, (scenario, values) in enumerate(series.items()):
            xs, ys = [], []
            for cat_idx, category in enumerate(categories):
                value = values.get(category)
                if value is None:
                    continue
                xs.append(cat_idx + (idx - (n - 1) / 2.0) * width)
                ys.append(value)
                rows.append([scenario, category, value])
            ax.bar(xs, ys, width=width, label=scenario)
        ax.set_xticks(range(len(categories)))
        ax.set_xticklabels(categories, rotation=15, ha='right')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, axis='y', alpha=0.3)
        ax.legend(fontsize=7)
        self.save(fig, basename, ['scenario', 'category', 'value'], rows)

    def overlaid_hist(self, basename, title, panels, data, unit_of):
        """One panel per signal, one translucent histogram per scenario.

        Scenarios share bin edges and are drawn as fractions, so bar heights
        compare across runs that logged a different number of samples.
        """
        fig, axes = self.plt.subplots(1, len(panels), figsize=(5.5 * len(panels), 4), squeeze=False)
        rows = []
        for ax, signal in zip(axes[0], panels):
            per_scenario = OrderedDict(
                (scenario, per_signal[signal])
                for scenario, per_signal in data.items() if per_signal.get(signal))
            for scenario, samples in per_scenario.items():
                rows.extend([scenario, signal, v] for v in samples)
            if not per_scenario:
                ax.set_title(f'{signal} (no data)')
                continue
            edges = _shared_bins(per_scenario.values())
            for scenario, samples in per_scenario.items():
                weights = [1.0 / len(samples)] * len(samples)
                ax.hist(samples, bins=edges, weights=weights, alpha=0.45, label=scenario)
            ax.set_title(signal)
            ax.set_xlabel(
                f'Error (GT - estimate) [{unit_of.get(signal, "")}] (central 99 %)')
            ax.set_ylabel('Fraction of samples')
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=7)
        fig.suptitle(title)
        self.save(fig, basename, ['scenario', 'signal', 'error_gt_minus_est'], rows)

    def series_grid(self, basename, title, panels, per_scenario_series, unit_of):
        """One row per scenario, one column per signal, each with its own truth.

        Scenarios are separate runs: they drive different laps for different
        durations, so their ground truths share nothing but t=0. Overlaying one
        run's estimate on another's truth would compare unrelated trajectories.
        """
        names = list(per_scenario_series)
        fig, axes = self.plt.subplots(
            len(names), len(panels), squeeze=False,
            figsize=(6.0 * len(panels), 3.2 * len(names)))
        rows = []
        for row_axes, scenario in zip(axes, names):
            getter = per_scenario_series[scenario]
            for ax, signal in zip(row_axes, panels):
                t, gt, est = getter(signal)
                if t:
                    ax.plot(t, gt, color='black', linewidth=1.0, alpha=0.6,
                            label='Ground truth')
                    ax.plot(t, est, linewidth=1.0, linestyle='--', label='Estimate')
                    rows.extend([scenario, signal, 'ground_truth', a, b]
                                for a, b in zip(t, gt))
                    rows.extend([scenario, signal, 'estimate', a, b]
                                for a, b in zip(t, est))
                    ax.legend(fontsize=7)
                ax.set_title(f'{scenario} - {signal}' if t
                             else f'{scenario} - {signal} (no data)')
                ax.set_xlabel('Time since first sample [s]')
                ax.set_ylabel(f'{signal} [{unit_of.get(signal, "")}]')
                ax.grid(True, alpha=0.3)
        fig.suptitle(title)
        self.save(fig, basename, ['scenario', 'signal', 'series', 't_s', 'value'], rows)

    def error_series(self, basename, title, panels, per_scenario_series, unit_of):
        """One panel per signal, one estimate-minus-truth trace per scenario.

        Errors share a meaning across runs even when the trajectories do not, so
        this is the one timeseries view that may overlay scenarios.
        """
        fig, axes = self.plt.subplots(len(panels), 1, figsize=(10, 3.2 * len(panels)),
                                      squeeze=False)
        rows = []
        for ax, signal in zip(axes[:, 0], panels):
            drew = False
            for scenario, getter in per_scenario_series.items():
                t, gt, est = getter(signal)
                if not t:
                    continue
                drew = True
                err = [e - g for e, g in zip(est, gt)]
                ax.plot(t, err, linewidth=1.0, label=scenario)
                rows.extend([scenario, signal, a, b] for a, b in zip(t, err))
            ax.axhline(0.0, color='black', linewidth=0.8, alpha=0.6)
            ax.set_title(signal if drew else f'{signal} (no data)')
            ax.set_xlabel('Time since first sample [s]')
            ax.set_ylabel(f'Estimate - truth [{unit_of.get(signal, "")}]')
            ax.grid(True, alpha=0.3)
            if drew:
                ax.legend(fontsize=7)
        fig.suptitle(title)
        self.save(fig, basename, ['scenario', 'signal', 't_s', 'error_est_minus_gt'], rows)


# ---------------- the comparison figures ----------------

def build(scenarios, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    # Overwrite means overwrite: the sweep that produced this directory may have
    # had more scenarios than the one producing it now.
    for stale in (list(out_dir.glob('*.png')) + list(out_dir.glob('*.pdf'))
                  + list(out_dir.glob('*.csv'))):
        stale.unlink()

    cmp = Comparison(
        out_dir, plt,
        incomplete=[(s.name, s.failure_reason) for s in scenarios if s.failure_reason])
    names = [s.name for s in scenarios]

    _summary_csv(scenarios, out_dir)
    _lap_times(cmp, scenarios)
    _tracking_error_box(cmp, scenarios)
    cmp.grouped_bars(
        'itae_by_scenario', 'ITAE of lateral error by active controller', 'ITAE e_y [m$\\cdot$s$^2$]',
        OrderedDict((s.name, {k: s.control('tracking', k, 'itae_e_y')
                              for k in ('Overall', 'RUNNING_PP', 'RUNNING_MPC')})
                    for s in scenarios),
        ['Overall', 'RUNNING_PP', 'RUNNING_MPC'])
    cmp.grouped_bars(
        'mpc_solve_time_by_scenario', 'MPC QP solve time', 'solve time [ms]',
        OrderedDict((s.name, {'RMS': s.control('summary', '', 'mpc_solve_time_rms_ms'),
                              'Max': s.control('summary', '', 'mpc_solve_time_max_ms')})
                    for s in scenarios),
        ['RMS', 'Max'])
    _e_y_hist(cmp, scenarios)

    cmp.grouped_bars(
        'tire_force_rmse_by_scenario', 'Tire lateral force: model accuracy per axle',
        'RMSE [N]',
        OrderedDict((s.name, {sig: s.ident(sig, 'RMSE') for sig in AXLE_FORCE_SIGNALS})
                    for s in scenarios),
        list(AXLE_FORCE_SIGNALS))
    cmp.grouped_bars(
        'tire_force_r2_by_scenario', 'Tire lateral force: $R^2$ per axle', '$R^2$ [-]',
        OrderedDict((s.name, {sig: s.ident(sig, 'R2') for sig in AXLE_FORCE_SIGNALS})
                    for s in scenarios),
        list(AXLE_FORCE_SIGNALS))
    cmp.overlaid_hist(
        'tire_force_error_hist_by_scenario', 'Tire lateral force error distribution by scenario',
        AXLE_FORCE_SIGNALS,
        OrderedDict((s.name, s.force_errors) for s in scenarios),
        {sig: 'N' for sig in AXLE_FORCE_SIGNALS})
    force_series = OrderedDict((s.name, (lambda sig, s=s: s.series(s.force_series, sig)))
                               for s in scenarios)
    cmp.series_grid(
        'tire_force_timeseries_by_scenario', 'Tire lateral force: estimate against truth',
        AXLE_FORCE_SIGNALS, force_series, {sig: 'N' for sig in AXLE_FORCE_SIGNALS})
    cmp.error_series(
        'tire_force_error_timeseries_by_scenario',
        'Tire lateral force error per scenario',
        AXLE_FORCE_SIGNALS, force_series, {sig: 'N' for sig in AXLE_FORCE_SIGNALS})
    _pacejka(cmp, scenarios)

    cmp.grouped_bars(
        'mu_error_by_scenario', 'Peak friction: identified $D$ against the simulator',
        'RMSE [-]',
        OrderedDict((s.name, {sig: s.ident(sig, 'RMSE') for sig in MU_SIGNALS})
                    for s in scenarios),
        list(MU_SIGNALS))
    _mu_series(cmp, scenarios)

    # Persistence baseline written by tire_force_benchmark: next state = last
    # measured state. The one-step prediction is evidence of a good tire model
    # only where it beats this.
    rmse_signals = tuple(s for sig in STATE_SIGNALS for s in (sig, f'{sig}_persistence'))
    cmp.grouped_bars(
        'state_rmse_by_scenario',
        'One-step state prediction accuracy against the persistence baseline', 'RMSE',
        OrderedDict((s.name, {sig: s.ident(sig, 'RMSE') for sig in rmse_signals})
                    for s in scenarios),
        list(rmse_signals))
    for signal in STATE_SIGNALS:
        cmp.overlaid_hist(
            f'state_error_hist_{signal}', f'{signal} prediction error by scenario',
            (signal,),
            OrderedDict((s.name, s.state_errors) for s in scenarios),
            STATE_UNITS)
    state_series = OrderedDict((s.name, (lambda sig, s=s: s.series(s.state_series, sig)))
                               for s in scenarios)
    cmp.series_grid(
        'state_timeseries_by_scenario', 'Estimated states against truth per scenario',
        STATE_SIGNALS, state_series, STATE_UNITS)
    cmp.error_series(
        'state_error_timeseries_by_scenario', 'State estimation error per scenario',
        STATE_SIGNALS, state_series, STATE_UNITS)

    print(f'wrote {len(cmp.written) + 1} comparison files to {out_dir}')
    print(f'  scenarios: {", ".join(names)}')


SUMMARY_COLUMNS = [
    ('laps', lambda s: s.control('summary', '', 'laps')),
    ('best_lap_s', lambda s: s.control('summary', '', 'best_lap_s')),
    ('mean_lap_s', lambda s: s.control('summary', '', 'mean_lap_s')),
    ('rms_e_y_m', lambda s: s.control('tracking', 'Overall', 'rms_e_y_m')),
    ('max_abs_e_y_m', lambda s: s.control('tracking', 'Overall', 'max_abs_e_y_m')),
    ('itae_e_y', lambda s: s.control('tracking', 'Overall', 'itae_e_y')),
    ('rms_heading_rad', lambda s: s.control('tracking', 'Overall', 'rms_heading_rad')),
    ('max_abs_heading_rad', lambda s: s.control('tracking', 'Overall', 'max_abs_heading_rad')),
    ('rms_e_y_pp_m', lambda s: s.control('tracking', 'RUNNING_PP', 'rms_e_y_m')),
    ('rms_e_y_mpc_m', lambda s: s.control('tracking', 'RUNNING_MPC', 'rms_e_y_m')),
    ('transitions', lambda s: s.control('summary', '', 'transitions')),
    ('emergency_halts', lambda s: s.control('summary', '', 'emergency_halts')),
    ('mpc_solve_time_rms_ms', lambda s: s.control('summary', '', 'mpc_solve_time_rms_ms')),
    ('mpc_solve_time_max_ms', lambda s: s.control('summary', '', 'mpc_solve_time_max_ms')),
    ('front_fy_rmse_N', lambda s: s.ident('front_sum_fy', 'RMSE')),
    ('rear_fy_rmse_N', lambda s: s.ident('rear_sum_fy', 'RMSE')),
    ('front_fy_r2', lambda s: s.ident('front_sum_fy', 'R2')),
    ('rear_fy_r2', lambda s: s.ident('rear_sum_fy', 'R2')),
    ('front_mu_rmse', lambda s: s.ident('front_mu', 'RMSE')),
    ('rear_mu_rmse', lambda s: s.ident('rear_mu', 'RMSE')),
    ('v_y_rmse_mps', lambda s: s.ident('v_y', 'RMSE')),
    ('omega_rmse_radps', lambda s: s.ident('omega', 'RMSE')),
    ('v_y_persistence_rmse_mps', lambda s: s.ident('v_y_persistence', 'RMSE')),
    ('omega_persistence_rmse_radps', lambda s: s.ident('omega_persistence', 'RMSE')),
    ('v_y_skill_vs_persistence', lambda s: _skill(s, 'v_y')),
    ('omega_skill_vs_persistence', lambda s: _skill(s, 'omega')),
]


def _skill(scenario, signal):
    """1 - RMSE_model / RMSE_persistence: positive only if the model helps."""
    model = scenario.ident(signal, 'RMSE')
    baseline = scenario.ident(f'{signal}_persistence', 'RMSE')
    if model is None or not baseline:
        return None
    return 1.0 - model / baseline


def _summary_csv(scenarios, out_dir):
    coeff_columns = [f'{axle}_{c}' for axle in ('front', 'rear') for c in 'BCDE']
    header = (['scenario', 'completed', 'failure_reason']
              + [name for name, _ in SUMMARY_COLUMNS] + coeff_columns)
    rows = []
    for s in scenarios:
        row = ([s.name, not s.failure_reason, s.failure_reason or '']
               + [getter(s) for _, getter in SUMMARY_COLUMNS])
        for axle in ('front', 'rear'):
            coefficients = s.pacejka_coefficients(axle)
            row.extend(coefficients if coefficients else ['', '', '', ''])
        rows.append(['' if v is None else v for v in row])
    with (out_dir / 'comparison_summary.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def _lap_times(cmp, scenarios):
    max_laps = max((len(s.lap_times) for s in scenarios), default=0)
    fig, ax = cmp.plt.subplots(figsize=(max(6, 1.1 * max_laps), 4))
    rows = []
    n = max(1, len(scenarios))
    width = 0.8 / n
    for idx, s in enumerate(scenarios):
        xs = [lap + (idx - (n - 1) / 2.0) * width for lap in range(1, len(s.lap_times) + 1)]
        ax.bar(xs, s.lap_times, width=width, label=s.name)
        rows.extend([s.name, lap, t] for lap, t in enumerate(s.lap_times, start=1))
    ax.set_xticks(range(1, max_laps + 1))
    ax.set_xlabel('Lap number')
    ax.set_ylabel('Lap time [s]')
    ax.set_title('Lap-wise lap time by scenario')
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend(fontsize=7)
    cmp.save(fig, 'lap_times_by_scenario', ['scenario', 'lap', 'lap_time_s'], rows)


def _tracking_error_box(cmp, scenarios):
    """|e_y| distribution, one box per (scenario, active controller)."""
    data, labels, rows = [], [], []
    for s in scenarios:
        for controller in ('RUNNING_PP', 'RUNNING_MPC'):
            samples = [float(r['abs_error']) for r in s.abs_error
                       if r['signal'] == '|e_y|' and r['controller'] == controller
                       and _is_float(r['abs_error'])]
            if not samples:
                continue
            data.append(samples)
            labels.append(f'{s.name}\n{controller.replace("RUNNING_", "")}')
            rows.extend([s.name, controller, v] for v in samples)
    fig, ax = cmp.plt.subplots(figsize=(max(6, 1.8 * max(1, len(data))), 4.5))
    if data:
        ax.boxplot(data, labels=labels)
        ax.set_ylabel('|e_y| [m]')
    else:
        ax.set_title('(no data)')
        ax.axis('off')
    ax.set_title('Lateral tracking error by scenario and active controller')
    ax.grid(True, axis='y', alpha=0.3)
    cmp.save(fig, 'tracking_error_by_scenario',
             ['scenario', 'controller', 'abs_e_y_m'], rows)


def _e_y_hist(cmp, scenarios):
    fig, ax = cmp.plt.subplots(figsize=(7, 4))
    rows = []
    per_scenario = OrderedDict()
    for s in scenarios:
        samples = [float(r['e_y_m']) for r in s.tracking_series if _is_float(r.get('e_y_m'))]
        rows.extend([s.name, v] for v in samples)
        if samples:
            per_scenario[s.name] = samples
    if per_scenario:
        edges = _shared_bins(per_scenario.values(), bins=50)
        for scenario, samples in per_scenario.items():
            weights = [1.0 / len(samples)] * len(samples)
            ax.hist(samples, bins=edges, weights=weights, alpha=0.45, label=scenario)
        ax.legend(fontsize=7)
    ax.set_xlabel('Signed lateral error e_y [m] (central 99 %)')
    ax.set_ylabel('Fraction of samples')
    ax.set_title('Lateral tracking error distribution by scenario')
    ax.grid(True, alpha=0.3)
    cmp.save(fig, 'e_y_error_hist_by_scenario', ['scenario', 'e_y_m'], rows)


def _pacejka(cmp, scenarios):
    """Every scenario's identified curve against the plant's nominal curve,
    plus the coefficients that actually differ between parameter sets."""
    fig, axes = cmp.plt.subplots(3, 1, figsize=(7.5, 11))
    rows = []
    for ax, axle in zip(axes[:2], ('front', 'rear')):
        # The nominal curve belongs to the plant, so it is drawn once when every
        # scenario reports the same one - and once per scenario when a friction
        # schedule moved mu and they no longer agree.
        curves = [(s.name,) + s.pacejka_curve(axle) for s in scenarios]
        curves = [c for c in curves if c[1]]
        shared_nominal = len({tuple(nominal) for _, _, _, nominal in curves}) == 1
        for idx, (name, alpha, identified, nominal) in enumerate(curves):
            if not shared_nominal:
                ax.plot(alpha, nominal, linewidth=2.0, alpha=0.6, label=f'{name} (nominal)')
                rows.extend([f'{name} (nominal)', axle, a, f] for a, f in zip(alpha, nominal))
            elif idx == 0:
                ax.plot(alpha, nominal, color='black', linewidth=2.0, label='Nominal (plant)')
                rows.extend(['(nominal)', axle, a, f] for a, f in zip(alpha, nominal))
            ax.plot(alpha, identified, linewidth=1.6, linestyle='--', label=name)
            rows.extend([name, axle, a, f] for a, f in zip(alpha, identified))
        drew_nominal = bool(curves)
        ax.set_title(f'{axle.capitalize()} axle')
        ax.set_xlabel(r'$\alpha$ [rad]')
        ax.set_ylabel(r'$F_y$ [N]')
        ax.grid(True, alpha=0.3)
        if drew_nominal:
            ax.legend(fontsize=7)
        else:
            ax.set_title(f'{axle.capitalize()} axle (no identification)')

    ax = axes[2]
    categories = [f'{axle}_{c}' for axle in ('front', 'rear') for c in 'BCDE']
    n = max(1, len(scenarios))
    width = 0.8 / n
    coeff_rows = []
    for idx, s in enumerate(scenarios):
        xs, ys = [], []
        for axle_idx, axle in enumerate(('front', 'rear')):
            coefficients = s.pacejka_coefficients(axle)
            if not coefficients:
                continue
            for c_idx, (label, value) in enumerate(zip('BCDE', coefficients)):
                position = axle_idx * 4 + c_idx
                xs.append(position + (idx - (n - 1) / 2.0) * width)
                ys.append(value)
                coeff_rows.append([s.name, f'{axle}_{label}', value])
        ax.bar(xs, ys, width=width, label=s.name)
    ax.set_xticks(range(len(categories)))
    ax.set_xticklabels(categories, rotation=30, ha='right')
    ax.set_ylabel('coefficient value')
    ax.set_title('Identified Magic Formula coefficients')
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend(fontsize=7)

    fig.suptitle('Identified vs. nominal tire model by scenario')
    cmp.save(fig, 'pacejka_identified_vs_nominal_by_scenario',
             ['scenario', 'series', 'alpha_rad_or_coefficient', 'value'],
             rows + coeff_rows)


def _mu_series(cmp, scenarios):
    """One row per scenario: identified D against that run's plant mu.

    Each scenario runs its own friction schedule for its own duration, so the
    scenarios share no time axis - only the plant mu of the same run belongs in
    one panel.
    """
    fig, axes = cmp.plt.subplots(
        len(scenarios), len(MU_SIGNALS), squeeze=False,
        figsize=(6.0 * len(MU_SIGNALS), 3.2 * len(scenarios)))
    rows = []
    for row_axes, s in zip(axes, scenarios):
        for ax, signal in zip(row_axes, MU_SIGNALS):
            t, gt, est = s.series(s.mu_series, signal)
            if t:
                ax.plot(t, gt, color='black', linewidth=1.0, alpha=0.6, label='Plant $\\mu$')
                ax.plot(t, est, linewidth=1.2, linestyle='--', label='Identified $D$')
                rows.extend([s.name, signal, 'plant_mu', a, b] for a, b in zip(t, gt))
                rows.extend([s.name, signal, 'identified_D', a, b] for a, b in zip(t, est))
            ax.set_title(f'{s.name} - {signal}' if t else f'{s.name} - {signal} (no data)')
            ax.set_xlabel('Time since first sample [s]')
            ax.set_ylabel('$\\mu$ / $D$ [-]')
            ax.grid(True, alpha=0.3)
            if t:
                ax.legend(fontsize=7)
    fig.suptitle('Peak friction tracking by scenario')
    cmp.save(fig, 'mu_timeseries_by_scenario',
             ['scenario', 'signal', 'series', 't_s', 'value'], rows)


def _shared_bins(sample_lists, bins=40, tail=0.5):
    """Common bin edges over the pooled central 99 % of every scenario's samples.

    Per-scenario edges would give each run its own bin width, and a single
    outlier would push all the mass into three bins.
    """
    import numpy as np
    pooled = np.concatenate([np.asarray(list(s), dtype=float) for s in sample_lists])
    lo, hi = np.percentile(pooled, [tail, 100.0 - tail])
    if not np.isfinite([lo, hi]).all() or hi <= lo:
        lo, hi = float(np.min(pooled)), float(np.max(pooled))
    if hi <= lo:
        hi = lo + 1e-9
    return np.linspace(lo, hi, bins + 1)


def _is_float(value):
    try:
        float(value)
        return True
    except (TypeError, ValueError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).resolve().parent / 'scenarios.yaml'))
    args = parser.parse_args()

    with Path(args.config).open() as handle:
        config = yaml.safe_load(handle)
    graphs_root = REPO_ROOT / config.get('graphs_root', 'graphs')

    scenarios = []
    for entry in config['scenarios']:
        data = ScenarioData(entry['name'], graphs_root)
        if data.exists():
            scenarios.append(data)
        else:
            print(f'skipping {entry["name"]}: no benchmark CSVs under {graphs_root}')
    if not scenarios:
        print('nothing to compare - run `make run_benchmark_scenarios` first')
        return 1

    for s in scenarios:
        if s.failure_reason:
            print(f'WARNING: {s.name} did not finish ({s.failure_reason}) - its '
                  f'partial data is still compared, and every figure says so.')

    build(scenarios, graphs_root / 'comparison')
    return 0


if __name__ == '__main__':
    sys.exit(main())
