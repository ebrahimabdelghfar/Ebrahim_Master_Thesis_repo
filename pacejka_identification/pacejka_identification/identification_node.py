#!/usr/bin/env python3
"""
ROS2 node that orchestrates Pacejka Magic Formula coefficient identification.

Phases:
  1. Data Collection  – subscribe to /sim/feedback/tire_forces, buffer samples
  2. Identification    – fit Pacejka [B,C,D,E] via nonlinear optimisation
  3. Publication       – publish identified coefficients and fit metrics

The force source is CARLA's own per-wheel telemetry
(sim_manager_msgs/TireForces): slip_angle, slip_ratio, normal_load,
lateral_force straight out of the simulator, so the identified coefficients
are a ground-truth fit of the physics the vehicle is actually driven by.
tire_friction (the effective mu the physics step uses, already multiplied by
the road surface) bounds what D may legitimately come out as, and is used here
as a cross-check on the fit.
"""

import math
import os
from datetime import datetime

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from sim_manager_msgs.msg import TireForces

from pacejka_identification.coefficient_identifier import CoefficientIdentifier


# ── Constants ────────────────────────────────────────────────────────────────
WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

PER_WHEEL_COLS = [
    'slip_angle', 'slip_ratio', 'fz', 'fy', 'fx', 'wheel_torque',
    'tire_friction', 'wheel_speed',
]


class IdentificationNode(Node):
    """
    All-in-one node: collects data, identifies Pacejka coefficients,
    publishes and exports results.
    """

    def __init__(self):
        super().__init__('identification_node')

        # ── Declare parameters ───────────────────────────────────────────
        self.declare_parameter('tire_forces_topic', '/sim/feedback/tire_forces')
        self.declare_parameter('data_collection.duration_seconds', 60)
        self.declare_parameter('data_collection.odom_topic', '/odom')
        self.declare_parameter('data_collection.min_speed', 2.0)
        self.declare_parameter('data_collection.min_fz_threshold', 50.0)

        self.declare_parameter('identification.method', 'dual')
        self.declare_parameter('identification.identification_mode', 'sequential')
        self.declare_parameter('identification.formulas', ['lateral_fy'])
        self.declare_parameter('identification.axle_grouping', 'per_axle')
        self.declare_parameter('identification.initial_guess_fy', [10.0, 1.5, 1.0, 0.5])
        self.declare_parameter('identification.initial_guess_fx', [10.0, 1.65, 1.0, 0.5])
        self.declare_parameter('identification.lower_bounds', [0.1, 0.1, 0.01, -2.0])
        self.declare_parameter('identification.upper_bounds', [50.0, 5.0, 5.0, 2.0])

        self.declare_parameter('output.csv_export', True)
        self.declare_parameter('output.csv_path', '')
        self.declare_parameter('output.yaml_export', True)
        self.declare_parameter('output.yaml_path', '')

        # Algorithm hyperparameters
        self.declare_parameter('trust_region.max_nfev', 10000)

        self.declare_parameter('differential_evolution.maxiter', 1000)
        self.declare_parameter('differential_evolution.tol', 1e-12)
        self.declare_parameter('differential_evolution.seed', 42)
        self.declare_parameter('differential_evolution.polish', True)

        self.declare_parameter('genetic_algorithm.pop_size', 120)
        self.declare_parameter('genetic_algorithm.n_generations', 400)
        self.declare_parameter('genetic_algorithm.crossover_rate', 0.85)
        self.declare_parameter('genetic_algorithm.mutation_rate', 0.15)
        self.declare_parameter('genetic_algorithm.mutation_scale', 0.10)
        self.declare_parameter('genetic_algorithm.elite_frac', 0.05)
        self.declare_parameter('genetic_algorithm.tournament_size', 3)
        self.declare_parameter('genetic_algorithm.seed', 42)

        self.declare_parameter('adaptive_de.pop_size', 100)
        self.declare_parameter('adaptive_de.n_generations', 400)
        self.declare_parameter('adaptive_de.p', 0.1)
        self.declare_parameter('adaptive_de.c', 0.1)
        self.declare_parameter('adaptive_de.archive_ratio', 1.0)
        self.declare_parameter('adaptive_de.seed', 42)

        self.declare_parameter('identification.regularization', 'none')
        self.declare_parameter('map_regularization.lambda_C', 1.0)
        self.declare_parameter('map_regularization.sigma_C', 0.3)

        self.declare_parameter('bayesian_svi.num_steps', 2000)
        self.declare_parameter('bayesian_svi.learning_rate', 0.01)
        self.declare_parameter('bayesian_svi.seed', 42)

        self.declare_parameter('data_balancing.enabled', False)
        self.declare_parameter('data_balancing.n_bins', 20)
        self.declare_parameter('data_balancing.max_per_bin', 200)

        # ── Read parameters ──────────────────────────────────────────────
        self.topic = self.get_parameter('tire_forces_topic').value
        self.duration = int(self.get_parameter('data_collection.duration_seconds').value)
        self.odom_topic = str(self.get_parameter('data_collection.odom_topic').value)
        self.min_speed = float(self.get_parameter('data_collection.min_speed').value)
        self.speed_gate_enabled = bool(self.odom_topic) and self.min_speed > 0.0
        self.min_fz = float(self.get_parameter('data_collection.min_fz_threshold').value)

        self.method = str(self.get_parameter('identification.method').value)
        self.id_mode = str(self.get_parameter('identification.identification_mode').value)
        self.formulas = list(self.get_parameter('identification.formulas').value)
        self.axle_grouping = str(self.get_parameter('identification.axle_grouping').value)
        self.ig_fy = [float(v) for v in self.get_parameter('identification.initial_guess_fy').value]
        self.ig_fx = [float(v) for v in self.get_parameter('identification.initial_guess_fx').value]
        self.lb = [float(v) for v in self.get_parameter('identification.lower_bounds').value]
        self.ub = [float(v) for v in self.get_parameter('identification.upper_bounds').value]

        self.csv_export = bool(self.get_parameter('output.csv_export').value)
        self.csv_path = str(self.get_parameter('output.csv_path').value)
        self.yaml_export = bool(self.get_parameter('output.yaml_export').value)
        self.yaml_path = str(self.get_parameter('output.yaml_path').value)

        # Read algorithm hyperparameters
        self.tr_params = {
            'max_nfev': int(self.get_parameter('trust_region.max_nfev').value),
        }
        self.de_params = {
            'maxiter': int(self.get_parameter('differential_evolution.maxiter').value),
            'tol': float(self.get_parameter('differential_evolution.tol').value),
            'seed': int(self.get_parameter('differential_evolution.seed').value),
            'polish': bool(self.get_parameter('differential_evolution.polish').value),
        }
        self.ga_params = {
            'pop_size': int(self.get_parameter('genetic_algorithm.pop_size').value),
            'n_generations': int(self.get_parameter('genetic_algorithm.n_generations').value),
            'crossover_rate': float(self.get_parameter('genetic_algorithm.crossover_rate').value),
            'mutation_rate': float(self.get_parameter('genetic_algorithm.mutation_rate').value),
            'mutation_scale': float(self.get_parameter('genetic_algorithm.mutation_scale').value),
            'elite_frac': float(self.get_parameter('genetic_algorithm.elite_frac').value),
            'tournament_size': int(self.get_parameter('genetic_algorithm.tournament_size').value),
            'seed': int(self.get_parameter('genetic_algorithm.seed').value),
        }
        self.jade_params = {
            'pop_size': int(self.get_parameter('adaptive_de.pop_size').value),
            'n_generations': int(self.get_parameter('adaptive_de.n_generations').value),
            'p': float(self.get_parameter('adaptive_de.p').value),
            'c': float(self.get_parameter('adaptive_de.c').value),
            'archive_ratio': float(self.get_parameter('adaptive_de.archive_ratio').value),
            'seed': int(self.get_parameter('adaptive_de.seed').value),
        }

        self.regularization = str(self.get_parameter('identification.regularization').value)
        self.map_reg_params = {
            'lambda_C': float(self.get_parameter('map_regularization.lambda_C').value),
            'sigma_C': float(self.get_parameter('map_regularization.sigma_C').value),
        }
        self.svi_params = {
            'num_steps': int(self.get_parameter('bayesian_svi.num_steps').value),
            'learning_rate': float(self.get_parameter('bayesian_svi.learning_rate').value),
            'seed': int(self.get_parameter('bayesian_svi.seed').value),
        }
        self.data_balancing_params = {
            'enabled': bool(self.get_parameter('data_balancing.enabled').value),
            'n_bins': int(self.get_parameter('data_balancing.n_bins').value),
            'max_per_bin': int(self.get_parameter('data_balancing.max_per_bin').value),
        }

        if 'self_aligning_mz' in self.formulas:
            self.get_logger().error(
                'self_aligning_mz requested but sim_manager_msgs/TireForces carries no '
                'self-aligning torque — dropping it.'
            )
            self.formulas = [f for f in self.formulas if f != 'self_aligning_mz']
        if 'longitudinal_fx' in self.formulas:
            self.get_logger().warn(
                'longitudinal_fx enabled: TireForces.longitudinal_force is not the force the '
                'chassis receives — it behaves as a friction-capacity report, pinned at or '
                'near tire_friction * normal_load and uncorrelated with m*a_x '
                '(corr +0.04). The Fx fit will recover that capacity envelope, not a tire '
                'curve. NOT ground truth.'
            )

        # ── Data storage ─────────────────────────────────────────────────
        self.data = {w: {c: [] for c in PER_WHEEL_COLS} for w in WHEEL_NAMES}
        self.sample_count = 0
        self.kept_count = 0
        self.t_start = None          # set on the first received message
        self._last_progress_t = 0.0
        self.speed = None            # latest |v| from odom, None until received
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

        # ── Subscribers ──────────────────────────────────────────────────
        self.sub = self.create_subscription(
            TireForces, self.topic, self._tire_cb, 10,
        )
        self.odom_sub = None
        if self.speed_gate_enabled:
            # The CARLA bridge publishes /odom BEST_EFFORT; a RELIABLE
            # subscription is QoS-incompatible and never receives a message.
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data,
            )

        # ── Timer (20 Hz) — orchestrates phases ──────────────────────────
        self.timer = self.create_timer(0.05, self._loop)

        self._publish_status('WAITING', 'Waiting for tire_forces data...')
        self.get_logger().info('IdentificationNode started')
        self.get_logger().info(f'  Topic:    {self.topic}')
        self.get_logger().info(
            f'  Speed gate: {self.odom_topic} >= {self.min_speed} m/s'
            if self.speed_gate_enabled else '  Speed gate: disabled'
        )
        self.get_logger().info(f'  Duration: {self.duration}s')
        self.get_logger().info(f'  Method:   {self.method}')
        self.get_logger().info(f'  ID Mode:  {self.id_mode}')
        self.get_logger().info(f'  Regularization: {self.regularization}')
        self.get_logger().info(f'  Formulas: {self.formulas}')
        self.get_logger().info(f'  Grouping: {self.axle_grouping}')

    # ══════════════════════════════════════════════════════════════════════
    # CALLBACKS
    # ══════════════════════════════════════════════════════════════════════

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def _tire_cb(self, msg: TireForces):
        """Buffer incoming per-wheel tire telemetry."""
        if self.collection_complete:
            return

        self.sample_count += 1
        if self.t_start is None:
            self.t_start = self.get_clock().now()

        if self.speed_gate_enabled and (self.speed is None or self.speed < self.min_speed):
            return

        # Message arrays are always ordered FL, FR, RL, RR; wheel_names carries
        # the same order, so honour it if the publisher ever changes.
        names = list(msg.wheel_names) if len(msg.wheel_names) == 4 else WHEEL_NAMES

        for i, wname in enumerate(names):
            if wname not in self.data:
                continue
            sample = {
                'slip_angle': msg.slip_angle[i],
                'slip_ratio': msg.slip_ratio[i],
                'fz': msg.normal_load[i],
                'fy': msg.lateral_force[i],
                'fx': msg.longitudinal_force[i],
                'wheel_torque': msg.wheel_torque[i],
                'tire_friction': msg.tire_friction[i],
                'wheel_speed': msg.wheel_speed[i],
            }
            if not self._valid(sample):
                continue
            for col, val in sample.items():
                self.data[wname][col].append(val)

        self.kept_count += 1

    def _valid(self, s: dict) -> bool:
        if abs(s['fz']) < self.min_fz:
            return False
        # Below 0.5 m/s the publisher zeroes slip_angle, slip_ratio and both
        # forces (PhysX sleeps a parked vehicle), while Fz stays live — so the
        # min_fz gate alone lets those empty samples through.
        if s['slip_angle'] == 0.0 and s['slip_ratio'] == 0.0 and s['fy'] == 0.0:
            return False
        if any(math.isnan(v) or math.isinf(v) for v in s.values()):
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
        """Collect for `duration` seconds of wall clock from the first message.

        The publish rate of /sim/feedback/tire_forces is not fixed, so the
        elapsed clock — not a sample count — defines the collection window.
        """
        if self.t_start is None:
            return

        elapsed = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
        pct = min(100.0, elapsed / self.duration * 100)

        if elapsed - self._last_progress_t >= max(1.0, self.duration / 20.0):
            self._last_progress_t = elapsed
            bar_len = 25
            filled = int(bar_len * pct / 100)
            bar = '=' * filled + '-' * (bar_len - filled)
            self.get_logger().info(
                f'Collecting: [{bar}] {pct:.1f}%  '
                f'({elapsed:.0f}/{self.duration}s, {self.kept_count}/{self.sample_count} msgs kept)'
            )
            self._publish_status('COLLECTING', f'{pct:.0f}%')
            if self.speed_gate_enabled and self.speed is None:
                self.get_logger().warn(
                    f'No {self.odom_topic} message yet — the speed gate is rejecting every '
                    'sample. Set data_collection.min_speed to 0.0 to disable it.'
                )

        if elapsed >= self.duration:
            self.collection_complete = True
            total = {w: len(self.data[w]['fy']) for w in WHEEL_NAMES}
            self.get_logger().info(
                f'Collection complete — {self.kept_count}/{self.sample_count} messages kept, '
                f'samples per wheel: {total}'
            )
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
            jade_params=self.jade_params,
            regularization=self.regularization,
            map_reg_params=self.map_reg_params,
            svi_params=self.svi_params,
            data_balancing_params=self.data_balancing_params,
        )

        all_results = {}

        groups = self._get_groups()

        for group_name, wheels in groups.items():
            group_res = {}

            # Pool data from the relevant wheels
            slip_angle = self._pool(wheels, 'slip_angle')
            slip_ratio = self._pool(wheels, 'slip_ratio')
            fz = self._pool(wheels, 'fz')
            fy = self._pool(wheels, 'fy')
            fx = self._pool(wheels, 'fx')

            mu_carla = self._pool(wheels, 'tire_friction')

            n = len(slip_angle)
            alpha_p99 = float(np.percentile(np.abs(slip_angle), 99)) if n else 0.0
            mu_peak = float(np.max(np.abs(fy) / np.maximum(fz, 1.0))) if n else 0.0
            # Effective mu the physics step is using (road surface already
            # folded in) — the ceiling any identified D has to respect.
            mu_sim = float(np.mean(mu_carla)) if n else 0.0
            self.get_logger().info(
                f'[{group_name}] n_samples={n}  p99|alpha|={alpha_p99:.4f} rad  '
                f'peak |Fy|/Fz={mu_peak:.3f}  sim tire_friction={mu_sim:.3f}'
            )
            if alpha_p99 < 0.03:
                self.get_logger().warn(
                    f'[{group_name}] Slip angles stay below 0.03 rad — only B*C*D is '
                    'identifiable there, so D (peak friction) will not be trustworthy. '
                    'Drive closer to the limit.'
                )

            if n < 20:
                self.get_logger().warn(f'[{group_name}] Not enough data ({n} < 20). Skipping.')
                continue

            # --- Lateral Fy ---
            if 'lateral_fy' in self.formulas:
                self._check_sign(group_name, slip_angle, fy)
                self.get_logger().info(f'[{group_name}] Identifying Fy ...')
                coeffs, metrics = identifier.identify_fy(slip_angle, fz, fy, self.ig_fy)
                group_res['Fy'] = {'B': coeffs[0], 'C': coeffs[1], 'D': coeffs[2], 'E': coeffs[3], **metrics}
                self.get_logger().info(
                    f'[{group_name}] Fy → B={coeffs[0]:.4f}, C={coeffs[1]:.4f}, '
                    f'D={coeffs[2]:.4f}, E={coeffs[3]:.4f}  R²={metrics["R2"]:.4f}  '
                    f'μ_data={metrics["mu_data"]:.4f}, μ_model={metrics["mu_model"]:.4f}'
                )
                if mu_sim > 0.0 and coeffs[2] > 1.05 * mu_sim:
                    self.get_logger().warn(
                        f'[{group_name}] D={coeffs[2]:.3f} exceeds the simulator\'s own '
                        f'effective friction {mu_sim:.3f} (tire_friction) — the fit claims '
                        'more grip than the physics step can deliver.'
                    )

            # --- Longitudinal Fx ---
            if 'longitudinal_fx' in self.formulas:
                # longitudinal_force saturates on tire_friction * Fz; report how
                # much of the data sits on that ceiling rather than on a curve.
                sat = float(np.mean(np.abs(fx) >= 0.95 * mu_carla * fz)) if n else 0.0
                self.get_logger().info(
                    f'[{group_name}] Identifying Fx ... ({sat * 100:.0f}% of samples at '
                    f'>=95% of tire_friction*Fz)'
                )
                coeffs, metrics = identifier.identify_fx(slip_ratio, fz, fx, self.ig_fx)
                group_res['Fx'] = {'B': coeffs[0], 'C': coeffs[1], 'D': coeffs[2], 'E': coeffs[3], **metrics}
                self.get_logger().info(
                    f'[{group_name}] Fx → B={coeffs[0]:.4f}, C={coeffs[1]:.4f}, '
                    f'D={coeffs[2]:.4f}, E={coeffs[3]:.4f}  R²={metrics["R2"]:.4f}  '
                    f'μ_data={metrics["mu_data"]:.4f}, μ_model={metrics["mu_model"]:.4f}'
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

    def _check_sign(self, group_name, slip, force):
        """Warn if slip and force disagree in sign.

        The Magic Formula here has D > 0, so it can only fit data where force
        rises with slip. CARLA's lateral_force is already flipped into the ROS
        body frame (+y = left) to match slip_angle; a negative correlation
        means that convention changed and the fit would collapse onto a bound.
        """
        if len(slip) < 20 or np.std(slip) == 0 or np.std(force) == 0:
            return
        corr = float(np.corrcoef(slip, force)[0, 1])
        if corr < 0.0:
            self.get_logger().warn(
                f'[{group_name}] corr(slip_angle, Fy) = {corr:.3f} < 0 — sign convention '
                'mismatch between slip and force. Negate one of them before trusting the fit.'
            )

    def _format_summary(self, results):
        lines = ['═══ Pacejka Identification Results ═══']
        for gname, gres in results.items():
            lines.append(f'── {gname} ──')
            for formula, vals in gres.items():
                b, c, d, e = vals['B'], vals['C'], vals['D'], vals['E']
                r2 = vals.get('R2', float('nan'))
                rmse = vals.get('RMSE', float('nan'))
                n = vals.get('n_samples', '?')
                mu_d = vals.get('mu_data', float('nan'))
                mu_m = vals.get('mu_model', float('nan'))
                lines.append(
                    f'  {formula}: B={b:.4f}, C={c:.4f}, D={d:.4f}, E={e:.4f}  '
                    f'| R²={r2:.4f}  RMSE={rmse:.4f}  n={n}  '
                    f'| μ_data={mu_d:.4f}  μ_model={mu_m:.4f}'
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
                'source_topic': self.topic,
                'source_msg': 'sim_manager_msgs/TireForces',
                'messages_received': self.sample_count,
                'messages_kept': self.kept_count,
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
                    'mu_data': vals.get('mu_data'),
                    'mu_model': vals.get('mu_model'),
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
