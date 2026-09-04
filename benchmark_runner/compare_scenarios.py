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

REPO_ROOT = Path(__file__).resolve().parent.parent

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
        self.mu_commanded = _read_rows(self.ident_dir / 'raw' / 'mu_commanded.csv')

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

    def __init__(self, out_dir, plt):
        self.out_dir = Path(out_dir)
        self.plt = plt
        self.written = []

    def save(self, fig, basename, header, rows):
        png = self.out_dir / f'{basename}.png'
        fig.tight_layout()
        fig.savefig(png, dpi=150)
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
        """One panel per signal, one translucent histogram per scenario."""
        fig, axes = self.plt.subplots(1, len(panels), figsize=(5.5 * len(panels), 4), squeeze=False)
        rows = []
        for ax, signal in zip(axes[0], panels):
            drew = False
            for scenario, per_signal in data.items():
                samples = per_signal.get(signal, [])
                if not samples:
                    continue
                drew = True
                ax.hist(samples, bins=40, alpha=0.45, label=scenario)
                rows.extend([scenario, signal, v] for v in samples)
            ax.set_title(signal)
            ax.set_xlabel(f'Error (GT - estimate) [{unit_of.get(signal, "")}]')
            ax.set_ylabel('Count')
            ax.grid(True, alpha=0.3)
            if drew:
                ax.legend(fontsize=7)
            else:
                ax.set_title(f'{signal} (no data)')
        fig.suptitle(title)
        self.save(fig, basename, ['scenario', 'signal', 'error_gt_minus_est'], rows)

    def overlaid_series(self, basename, title, panels, per_scenario_series, unit_of):
        """Ground truth once (same plant), one estimate trace per scenario."""
        fig, axes = self.plt.subplots(len(panels), 1, figsize=(10, 3.2 * len(panels)),
                                      squeeze=False)
        rows = []
        for ax, signal in zip(axes[:, 0], panels):
            drew_gt = False
            for scenario, getter in per_scenario_series.items():
                t, gt, est = getter(signal)
                if not t:
                    continue
                if not drew_gt:
                    ax.plot(t, gt, color='black', linewidth=1.0, alpha=0.6,
                            label='Ground truth')
                    rows.extend(['(ground truth)', signal, a, b] for a, b in zip(t, gt))
                    drew_gt = True
                ax.plot(t, est, linewidth=1.0, linestyle='--', label=scenario)
                rows.extend([scenario, signal, a, b] for a, b in zip(t, est))
            ax.set_title(signal if drew_gt else f'{signal} (no data)')
            ax.set_xlabel('Time since first sample [s]')
            ax.set_ylabel(f'{signal} [{unit_of.get(signal, "")}]')
            ax.grid(True, alpha=0.3)
            if drew_gt:
                ax.legend(fontsize=7)
        fig.suptitle(title)
        self.save(fig, basename, ['scenario', 'signal', 't_s', 'value'], rows)


# ---------------- the comparison figures ----------------

def build(scenarios, out_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)
    # Overwrite means overwrite: the sweep that produced this directory may have
    # had more scenarios than the one producing it now.
    for stale in list(out_dir.glob('*.png')) + list(out_dir.glob('*.csv')):
        stale.unlink()

    cmp = Comparison(out_dir, plt)
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
    cmp.overlaid_series(
        'tire_force_timeseries_by_scenario', 'Tire lateral force: estimate per scenario',
        AXLE_FORCE_SIGNALS,
        OrderedDict((s.name, (lambda sig, s=s: s.series(s.force_series, sig)))
                    for s in scenarios),
        {sig: 'N' for sig in AXLE_FORCE_SIGNALS})
    _pacejka(cmp, scenarios)

    cmp.grouped_bars(
        'mu_error_by_scenario', 'Peak friction: identified $D$ against the simulator',
        'RMSE [-]',
        OrderedDict((s.name, {sig: s.ident(sig, 'RMSE') for sig in MU_SIGNALS})
                    for s in scenarios),
        list(MU_SIGNALS))
    _mu_series(cmp, scenarios)

    cmp.grouped_bars(
        'state_rmse_by_scenario', 'One-step state prediction accuracy', 'RMSE',
        OrderedDict((s.name, {sig: s.ident(sig, 'RMSE') for sig in STATE_SIGNALS})
                    for s in scenarios),
        list(STATE_SIGNALS))
    for signal in STATE_SIGNALS:
        cmp.overlaid_hist(
            f'state_error_hist_{signal}', f'{signal} prediction error by scenario',
            (signal,),
            OrderedDict((s.name, s.state_errors) for s in scenarios),
            STATE_UNITS)
    cmp.overlaid_series(
        'state_timeseries_by_scenario', 'Estimated states per scenario', STATE_SIGNALS,
        OrderedDict((s.name, (lambda sig, s=s: s.series(s.state_series, sig)))
                    for s in scenarios),
        STATE_UNITS)

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
]


def _summary_csv(scenarios, out_dir):
    coeff_columns = [f'{axle}_{c}' for axle in ('front', 'rear') for c in 'BCDE']
    header = ['scenario'] + [name for name, _ in SUMMARY_COLUMNS] + coeff_columns
    rows = []
    for s in scenarios:
        row = [s.name] + [getter(s) for _, getter in SUMMARY_COLUMNS]
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
    drew = False
    for s in scenarios:
        samples = [float(r['e_y_m']) for r in s.tracking_series if _is_float(r.get('e_y_m'))]
        if not samples:
            continue
        drew = True
        ax.hist(samples, bins=50, alpha=0.45, label=s.name)
        rows.extend([s.name, v] for v in samples)
    ax.set_xlabel('Signed lateral error e_y [m]')
    ax.set_ylabel('Count')
    ax.set_title('Lateral tracking error distribution by scenario')
    ax.grid(True, alpha=0.3)
    if drew:
        ax.legend(fontsize=7)
    cmp.save(fig, 'e_y_error_hist_by_scenario', ['scenario', 'e_y_m'], rows)


def _pacejka(cmp, scenarios):
    """Every scenario's identified curve against the one shared nominal curve,
    plus the coefficients that actually differ between parameter sets."""
    fig, axes = cmp.plt.subplots(3, 1, figsize=(7.5, 11))
    rows = []
    for ax, axle in zip(axes[:2], ('front', 'rear')):
        drew_nominal = False
        for s in scenarios:
            alpha, identified, nominal = s.pacejka_curve(axle)
            if not alpha:
                continue
            if not drew_nominal:
                # Same plant in every scenario, so one nominal curve is enough -
                # unless a friction schedule moved mu, which the caveat in the
                # README covers.
                ax.plot(alpha, nominal, color='black', linewidth=2.0, label='Nominal (plant)')
                rows.extend(['(nominal)', axle, a, f] for a, f in zip(alpha, nominal))
                drew_nominal = True
            ax.plot(alpha, identified, linewidth=1.6, linestyle='--', label=s.name)
            rows.extend([s.name, axle, a, f] for a, f in zip(alpha, identified))
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
    """Identified D(t) against the friction the runner commanded."""
    fig, axes = cmp.plt.subplots(len(MU_SIGNALS), 1, figsize=(10, 3.2 * len(MU_SIGNALS)),
                                 squeeze=False)
    rows = []
    for ax, signal in zip(axes[:, 0], MU_SIGNALS):
        drew = False
        for s in scenarios:
            t, gt, est = s.series(s.mu_series, signal)
            if not t:
                continue
            drew = True
            ax.plot(t, gt, linewidth=1.0, alpha=0.6, label=f'{s.name} (plant $\\mu$)')
            ax.plot(t, est, linewidth=1.2, linestyle='--', label=f'{s.name} (identified $D$)')
            rows.extend([s.name, signal, 'plant_mu', a, b] for a, b in zip(t, gt))
            rows.extend([s.name, signal, 'identified_D', a, b] for a, b in zip(t, est))
        for s in scenarios:
            commanded = [(float(r['t_s']), float(r['commanded_tire_friction']))
                         for r in s.mu_commanded
                         if _is_float(r.get('t_s')) and _is_float(r.get('commanded_tire_friction'))]
            if not commanded:
                continue
            rows.extend([s.name, signal, 'commanded_tire_friction', t, v] for t, v in commanded)
        ax.set_title(signal if drew else f'{signal} (no data)')
        ax.set_xlabel('Time since first sample [s]')
        ax.set_ylabel('$\\mu$ / $D$ [-]')
        ax.grid(True, alpha=0.3)
        if drew:
            ax.legend(fontsize=6, ncol=2)
    fig.suptitle('Peak friction tracking by scenario')
    cmp.save(fig, 'mu_timeseries_by_scenario',
             ['scenario', 'signal', 'series', 't_s', 'value'], rows)


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

    build(scenarios, graphs_root / 'comparison')
    return 0


if __name__ == '__main__':
    sys.exit(main())
