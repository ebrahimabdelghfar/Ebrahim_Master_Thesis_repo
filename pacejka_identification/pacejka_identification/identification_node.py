#!/usr/bin/env python3
"""
ROS2 node that orchestrates Pacejka Magic Formula coefficient identification.

Phases:
  1. Data Collection  – subscribe to /tire_forces, buffer samples
  2. Identification    – fit Pacejka [B,C,D,E] via nonlinear optimisation
  3. Publication       – publish identified coefficients and fit metrics
"""

import math
import os
from datetime import datetime

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String

from hellocm_msgs.msg import TireForcesArray

from pacejka_identification.coefficient_identifier import CoefficientIdentifier


# ── Constants ────────────────────────────────────────────────────────────────
WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

PER_WHEEL_COLS = [
    'slip_angle', 'long_slip', 'fz', 'fy', 'fx', 'mz',
    'inclination_angle', 'mu_road',
]


class IdentificationNode(Node):
    """
    All-in-one node: collects data, identifies Pacejka coefficients,
    publishes and exports results.
    """

    def __init__(self):
        super().__init__('identification_node')

        # ── Declare parameters ───────────────────────────────────────────
        self.declare_parameter('tire_forces_topic', '/tire_forces')
        self.declare_parameter('duration_seconds', 60)
        self.declare_parameter('min_velocity', 0.5)
        self.declare_parameter('require_on_road', True)
        self.declare_parameter('min_fz_threshold', 50.0)

        self.declare_parameter('method', 'dual')
        self.declare_parameter('identification_mode', 'sequential')
        self.declare_parameter('formulas', ['lateral_fy', 'longitudinal_fx', 'self_aligning_mz'])
        self.declare_parameter('axle_grouping', 'per_axle')
        self.declare_parameter('initial_guess_fy', [10.0, 1.5, 1.0, 0.5])
        self.declare_parameter('initial_guess_fx', [10.0, 1.65, 1.0, 0.5])
        self.declare_parameter('initial_guess_mz', [10.0, 1.5, 0.1, 0.5])
        self.declare_parameter('lower_bounds', [0.1, 0.1, 0.01, -2.0])
        self.declare_parameter('upper_bounds', [50.0, 5.0, 5.0, 2.0])

        self.declare_parameter('csv_export', True)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('yaml_export', True)
        self.declare_parameter('yaml_path', '')

        # Algorithm hyperparameters
        self.declare_parameter('tr_max_nfev', 10000)

        self.declare_parameter('de_maxiter', 1000)
        self.declare_parameter('de_tol', 1e-12)
        self.declare_parameter('de_seed', 42)
        self.declare_parameter('de_polish', True)

        self.declare_parameter('ga_pop_size', 120)
        self.declare_parameter('ga_n_generations', 400)
        self.declare_parameter('ga_crossover_rate', 0.85)
        self.declare_parameter('ga_mutation_rate', 0.15)
        self.declare_parameter('ga_mutation_scale', 0.10)
        self.declare_parameter('ga_elite_frac', 0.05)
        self.declare_parameter('ga_tournament_size', 3)
        self.declare_parameter('ga_seed', 42)

        # ── Read parameters ──────────────────────────────────────────────
        self.topic = self.get_parameter('tire_forces_topic').value
        self.duration = int(self.get_parameter('duration_seconds').value)
        self.min_vel = float(self.get_parameter('min_velocity').value)
        self.require_on_road = bool(self.get_parameter('require_on_road').value)
        self.min_fz = float(self.get_parameter('min_fz_threshold').value)

        self.method = str(self.get_parameter('method').value)
        self.id_mode = str(self.get_parameter('identification_mode').value)
        self.formulas = list(self.get_parameter('formulas').value)
        self.axle_grouping = str(self.get_parameter('axle_grouping').value)
        self.ig_fy = [float(v) for v in self.get_parameter('initial_guess_fy').value]
        self.ig_fx = [float(v) for v in self.get_parameter('initial_guess_fx').value]
        self.ig_mz = [float(v) for v in self.get_parameter('initial_guess_mz').value]
        self.lb = [float(v) for v in self.get_parameter('lower_bounds').value]
        self.ub = [float(v) for v in self.get_parameter('upper_bounds').value]

        self.csv_export = bool(self.get_parameter('csv_export').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.yaml_export = bool(self.get_parameter('yaml_export').value)
        self.yaml_path = str(self.get_parameter('yaml_path').value)

        # Read algorithm hyperparameters
        self.tr_params = {
            'max_nfev': int(self.get_parameter('tr_max_nfev').value),
        }
        self.de_params = {
            'maxiter': int(self.get_parameter('de_maxiter').value),
            'tol': float(self.get_parameter('de_tol').value),
            'seed': int(self.get_parameter('de_seed').value),
            'polish': bool(self.get_parameter('de_polish').value),
        }
        self.ga_params = {
            'pop_size': int(self.get_parameter('ga_pop_size').value),
            'n_generations': int(self.get_parameter('ga_n_generations').value),
            'crossover_rate': float(self.get_parameter('ga_crossover_rate').value),
            'mutation_rate': float(self.get_parameter('ga_mutation_rate').value),
            'mutation_scale': float(self.get_parameter('ga_mutation_scale').value),
            'elite_frac': float(self.get_parameter('ga_elite_frac').value),
            'tournament_size': int(self.get_parameter('ga_tournament_size').value),
            'seed': int(self.get_parameter('ga_seed').value),
        }

        # ── Data storage ─────────────────────────────────────────────────
        self.data = {w: {c: [] for c in PER_WHEEL_COLS} for w in WHEEL_NAMES}
        self.sample_count = 0
        self.expected_samples = self.duration * 100  # 100 Hz
        self.collection_complete = False
        self.identification_complete = False

        # Identified results
        self.results = {}

        # ── Publishers ───────────────────────────────────────────────────
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.coeff_pub = self.create_publisher(String, '/pacejka_id/coefficients', qos_latched)
        self.metrics_pub = self.create_publisher(String, '/pacejka_id/fit_metrics', qos_latched)
        self.status_pub = self.create_publisher(String, '/pacejka_id/status', 10)

        # ── Subscriber ───────────────────────────────────────────────────
        self.sub = self.create_subscription(
            TireForcesArray, self.topic, self._tire_cb, 10,
        )

        # ── Timer (20 Hz) — orchestrates phases ──────────────────────────
        self.timer = self.create_timer(0.05, self._loop)

        self._publish_status('WAITING', 'Waiting for tire_forces data...')
        self.get_logger().info('IdentificationNode started')
        self.get_logger().info(f'  Topic:    {self.topic}')
        self.get_logger().info(f'  Duration: {self.duration}s')
        self.get_logger().info(f'  Method:   {self.method}')
        self.get_logger().info(f'  ID Mode:  {self.id_mode}')
        self.get_logger().info(f'  Formulas: {self.formulas}')
        self.get_logger().info(f'  Grouping: {self.axle_grouping}')

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════════

    def _tire_cb(self, msg: TireForcesArray):
        """Buffer incoming tire data."""
        if self.collection_complete:
            return

        tires = [msg.front_left, msg.front_right, msg.rear_left, msg.rear_right]

        for wname, tire in zip(WHEEL_NAMES, tires):
            if not self._valid(tire):
                continue
            d = self.data[wname]
            d['slip_angle'].append(tire.slip_angle)
            d['long_slip'].append(tire.long_slip)
            d['fz'].append(tire.fz)
            d['fy'].append(tire.fy)
            d['fx'].append(tire.fx)
            d['mz'].append(tire.mz)
            d['inclination_angle'].append(tire.inclination_angle)
            d['mu_road'].append(tire.mu_road)

        self.sample_count += 1

    def _valid(self, t) -> bool:
        if self.require_on_road and not t.on_road:
            return False
        if abs(t.fz) < self.min_fz:
            return False
        if t.belt_velocity < self.min_vel:
            return False
        if any(math.isnan(v) for v in [t.slip_angle, t.long_slip, t.fz, t.fy, t.fx, t.mz]):
            return False
        return True

    # ══════════════════════════════════════════════════════════════════════
    # MAIN LOOP
    # ══════════════════════════════════════════════════════════════════════

    def _loop(self):
        if not self.collection_complete:
            self._phase_collection()
        elif not self.identification_complete:
            self._phase_identification()
        # After identification, the node keeps running (latched topics)

    # ── Phase 1: Data Collection ─────────────────────────────────────────

    def _phase_collection(self):
        interval = max(1, self.expected_samples // 20)
        if self.sample_count > 0 and self.sample_count % interval == 0:
            pct = min(100.0, self.sample_count / self.expected_samples * 100)
            bar_len = 25
            filled = int(bar_len * pct / 100)
            bar = '=' * filled + '-' * (bar_len - filled)
            self.get_logger().info(
                f'Collecting: [{bar}] {pct:.1f}%  ({self.sample_count}/{self.expected_samples})'
            )
            self._publish_status('COLLECTING', f'{pct:.0f}%')

        if self.sample_count >= self.expected_samples:
            self.collection_complete = True
            total = {w: len(self.data[w]['fy']) for w in WHEEL_NAMES}
            self.get_logger().info(f'Collection complete — samples per wheel: {total}')
            self._publish_status('COLLECTION_DONE', str(total))

            if self.csv_export:
                self._export_csv()

    # ── Phase 2: Identification ──────────────────────────────────────────

    def _phase_identification(self):
        self._publish_status('IDENTIFYING', f'method={self.method}')
        self.get_logger().info(f'Starting coefficient identification (method={self.method})...')

        identifier = CoefficientIdentifier(
            method=self.method,
            identification_mode=self.id_mode,
            lower_bounds=self.lb,
            upper_bounds=self.ub,
            tr_params=self.tr_params,
            de_params=self.de_params,
            ga_params=self.ga_params,
        )

        all_results = {}

        groups = self._get_groups()

        for group_name, wheels in groups.items():
            group_res = {}

            # Pool data from the relevant wheels
            slip_angle = self._pool(wheels, 'slip_angle')
            long_slip = self._pool(wheels, 'long_slip')
            fz = self._pool(wheels, 'fz')
            fy = self._pool(wheels, 'fy')
            fx = self._pool(wheels, 'fx')
            mz = self._pool(wheels, 'mz')

            n = len(slip_angle)
            self.get_logger().info(f'[{group_name}] n_samples={n}')

            if n < 20:
                self.get_logger().warn(f'[{group_name}] Not enough data ({n} < 20). Skipping.')
                continue

            # --- Lateral Fy ---
            if 'lateral_fy' in self.formulas:
                self.get_logger().info(f'[{group_name}] Identifying Fy ...')
                coeffs, metrics = identifier.identify_fy(slip_angle, fz, fy, self.ig_fy)
                group_res['Fy'] = {'B': coeffs[0], 'C': coeffs[1], 'D': coeffs[2], 'E': coeffs[3], **metrics}
                self.get_logger().info(
                    f'[{group_name}] Fy → B={coeffs[0]:.4f}, C={coeffs[1]:.4f}, '
                    f'D={coeffs[2]:.4f}, E={coeffs[3]:.4f}  R²={metrics["R2"]:.4f}'
                )

            # --- Longitudinal Fx ---
            if 'longitudinal_fx' in self.formulas:
                self.get_logger().info(f'[{group_name}] Identifying Fx ...')
                coeffs, metrics = identifier.identify_fx(long_slip, fz, fx, self.ig_fx)
                group_res['Fx'] = {'B': coeffs[0], 'C': coeffs[1], 'D': coeffs[2], 'E': coeffs[3], **metrics}
                self.get_logger().info(
                    f'[{group_name}] Fx → B={coeffs[0]:.4f}, C={coeffs[1]:.4f}, '
                    f'D={coeffs[2]:.4f}, E={coeffs[3]:.4f}  R²={metrics["R2"]:.4f}'
                )

            # --- Self-aligning Mz ---
            if 'self_aligning_mz' in self.formulas:
                self.get_logger().info(f'[{group_name}] Identifying Mz ...')
                coeffs, metrics = identifier.identify_mz(slip_angle, fz, mz, self.ig_mz)
                group_res['Mz'] = {'B': coeffs[0], 'C': coeffs[1], 'D': coeffs[2], 'E': coeffs[3], **metrics}
                self.get_logger().info(
                    f'[{group_name}] Mz → B={coeffs[0]:.4f}, C={coeffs[1]:.4f}, '
                    f'D={coeffs[2]:.4f}, E={coeffs[3]:.4f}  R²={metrics["R2"]:.4f}'
                )

            all_results[group_name] = group_res

        self.results = all_results
        self.identification_complete = True

        # Publish
        coeff_msg = String()
        coeff_msg.data = yaml.dump(all_results, default_flow_style=False)
        self.coeff_pub.publish(coeff_msg)

        # Publish a compact summary as well
        summary_lines = self._format_summary(all_results)
        self.get_logger().info('\n' + '\n'.join(summary_lines))

        metrics_msg = String()
        metrics_msg.data = '\n'.join(summary_lines)
        self.metrics_pub.publish(metrics_msg)

        self._publish_status('DONE', 'Identification complete')

        # Export to YAML
        if self.yaml_export:
            self._export_yaml(all_results)

    # ══════════════════════════════════════════════════════════════════════
    # HELPERS
    # ══════════════════════════════════════════════════════════════════════

    def _get_groups(self):
        """Return dict of group_name → list of wheel names."""
        if self.axle_grouping == 'per_wheel':
            return {w: [w] for w in WHEEL_NAMES}
        elif self.axle_grouping == 'per_axle':
            return {'front': ['FL', 'FR'], 'rear': ['RL', 'RR']}
        else:  # combined
            return {'all': WHEEL_NAMES}

    def _pool(self, wheels, col):
        """Pool data from multiple wheels."""
        return np.concatenate([
            np.array(self.data[w][col], dtype=np.float64) for w in wheels
        ])

    def _format_summary(self, results):
        lines = ['═══ Pacejka Identification Results ═══']
        for gname, gres in results.items():
            lines.append(f'── {gname} ──')
            for formula, vals in gres.items():
                b, c, d, e = vals['B'], vals['C'], vals['D'], vals['E']
                r2 = vals.get('R2', float('nan'))
                rmse = vals.get('RMSE', float('nan'))
                n = vals.get('n_samples', '?')
                lines.append(
                    f'  {formula}: B={b:.4f}, C={c:.4f}, D={d:.4f}, E={e:.4f}  '
                    f'| R²={r2:.4f}  RMSE={rmse:.4f}  n={n}'
                )
        lines.append('═════════════════════════════════════')
        return lines

    def _publish_status(self, phase, detail=''):
        msg = String()
        msg.data = f'{phase}: {detail}'
        self.status_pub.publish(msg)

    # ── CSV export ───────────────────────────────────────────────────────

    def _export_csv(self):
        import csv as csv_mod

        path = self.csv_path
        if not path:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.expanduser(f'~/pacejka_id_data_{ts}.csv')

        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        headers = []
        for w in WHEEL_NAMES:
            for c in PER_WHEEL_COLS:
                headers.append(f'{w}_{c}')

        min_len = min(len(self.data[w]['slip_angle']) for w in WHEEL_NAMES)

        with open(path, 'w', newline='') as f:
            writer = csv_mod.writer(f)
            writer.writerow(headers)
            for i in range(min_len):
                row = []
                for w in WHEEL_NAMES:
                    for c in PER_WHEEL_COLS:
                        row.append(self.data[w][c][i])
                writer.writerow(row)

        self.get_logger().info(f'Dataset CSV exported to: {path}')

    # ── YAML export ──────────────────────────────────────────────────────

    def _export_yaml(self, results):
        path = self.yaml_path
        if not path:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.expanduser(f'~/pacejka_id_coefficients_{ts}.yaml')

        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        # Build a clean exportable dict
        export = {
            'identification': {
                'method': self.method,
                'formulas': self.formulas,
                'axle_grouping': self.axle_grouping,
                'timestamp': datetime.now().isoformat(),
                'total_samples': self.sample_count,
            },
            'coefficients': {},
        }

        for gname, gres in results.items():
            export['coefficients'][gname] = {}
            for formula, vals in gres.items():
                export['coefficients'][gname][formula] = {
                    'B': vals['B'],
                    'C': vals['C'],
                    'D': vals['D'],
                    'E': vals['E'],
                    'params': [vals['B'], vals['C'], vals['D'], vals['E']],
                    'R2': vals.get('R2'),
                    'RMSE': vals.get('RMSE'),
                    'n_samples': vals.get('n_samples'),
                }

        with open(path, 'w') as f:
            yaml.dump(export, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'Coefficients YAML exported to: {path}')


# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = IdentificationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
