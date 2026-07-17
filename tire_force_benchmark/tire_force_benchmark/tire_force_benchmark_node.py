#!/usr/bin/env python3

import csv
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import rclpy.time
import yaml
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

from ackermann_msgs.msg import AckermannDriveStamped
from adaptive_controller_interfaces.srv import IdentifiedParam
from nav_msgs.msg import Odometry

try:
    # hellocm_msgs is IPG CarMaker's own message package - only present in a
    # CarMaker-based workspace, not e.g. an f1tenth_simulator one. Imported
    # lazily so a workspace without it can still run this node with
    # enable_force_benchmarking:=false (state-only benchmarking) instead of
    # crashing on import before parameters are even read.
    from hellocm_msgs.msg import TireForcesArray
    _HELLOCM_MSGS_AVAILABLE = True
except ImportError:
    TireForcesArray = None
    _HELLOCM_MSGS_AVAILABLE = False

from tire_force_benchmark.online_metrics import HistoryBuffer
from tire_force_benchmark.online_metrics import OnlineBenchmark

FORCE_SIGNALS = [
    ('fl_fy', 'FL Fy', 'N'),
    ('fr_fy', 'FR Fy', 'N'),
    ('rl_fy', 'RL Fy', 'N'),
    ('rr_fy', 'RR Fy', 'N'),
    ('front_sum_fy', 'Front axle Fy sum', 'N'),
    ('rear_sum_fy', 'Rear axle Fy sum', 'N'),
    ('total_sum_fy', 'Vehicle total Fy sum', 'N'),
]
STATE_SIGNALS = [
    ('v_y', 'Lateral velocity v_y', 'm/s'),
    ('omega', 'Yaw rate omega', 'rad/s'),
]
G = 9.81

# Fixed roles (not per-series-index) so ground truth / estimate keep the same
# color across every figure: blue/red, the categorical pair validated for
# CVD-safe adjacent contrast in both light and dark surfaces (see the
# dataviz palette reference). Academic figures here use a light, printable
# surface only.
COLOR_GT = '#2a78d6'
COLOR_EST = '#e34948'
COLOR_NOMINAL = '#4a3aa7'
COLOR_GRID = '#e1e0d9'
COLOR_INK = '#0b0b0b'
COLOR_SECONDARY_INK = '#52514e'
COLOR_MUTED = '#898781'


def pacejka_formula(params, alpha, fz):
    b, c, d, e = params[0], params[1], params[2], params[3]
    return fz * d * np.sin(c * np.arctan(b * alpha - e * (b * alpha - np.arctan(b * alpha))))


class TireForceBenchmarkNode(Node):
    def __init__(self):
        super().__init__('tire_force_benchmark_node')

        self.declare_parameter('enable_force_benchmarking', True)
        self.declare_parameter('benchmark_mode', 'internal_pacejka')
        self.declare_parameter('tire_forces_topic', '/tire_forces')
        self.declare_parameter('estimated_fy_topic', '/estimated_tire_force_fy')
        self.declare_parameter('external_prediction_lead_samples', 1)
        self.declare_parameter('external_max_queue_size', 2000)
        self.declare_parameter('log_interval', 200)
        # No default: a wrong-for-this-vehicle default would silently reject
        # every sample (see docs/tire_force_benchmark.md). Must be set explicitly.
        self.declare_parameter('min_fz_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('require_on_road', True)
        self.declare_parameter('model_file', '')
        # No default: benchmarking a hardcoded/arbitrary Pacejka model isn't
        # meaningful. c_pf/c_pr come from model_file (below) or from the
        # identified_params_service (see handle_identified_params) - until
        # one of those provides them, internal_pacejka benchmarking and
        # state-prediction are held off entirely (self.have_identified_params).
        self.declare_parameter('c_pf', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('c_pr', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('identified_params_service', 'benchmark/update_params')
        # Purely a plotting reference (see _plot_identified_vs_nominal) - unlike
        # model_file above, this never seeds c_pf/c_pr/have_identified_params, so
        # it can point at a stale/prior model (e.g. On-Track-SysID's previous-run
        # *_pacejka.txt) without contaminating the live benchmark it's compared
        # against.
        self.declare_parameter('nominal_model_file', '')
        self.declare_parameter('csv_output_path', '')

        self.declare_parameter('enable_state_benchmarking', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ackermann_cmd_topic', '/drive')
        self.declare_parameter('state_estimate_topic', '/benchmarking/state_estimate')
        self.declare_parameter('state_sensor_topic', '/benchmarking/state_sensor')
        self.declare_parameter('state_error_topic', '/benchmarking/state_error')
        self.declare_parameter('m', 0.0)
        self.declare_parameter('I_z', 0.0)
        self.declare_parameter('l_f', 0.0)
        self.declare_parameter('l_r', 0.0)
        self.declare_parameter('l_wb', 0.0)

        self.declare_parameter('plot_output_dir', '')
        self.declare_parameter('plot_max_points', 5000)

        self.enable_force_benchmarking = bool(self.get_parameter('enable_force_benchmarking').value)
        self.benchmark_mode = str(self.get_parameter('benchmark_mode').value).strip()
        tire_forces_topic = self.get_parameter('tire_forces_topic').value
        self.estimated_fy_topic = str(self.get_parameter('estimated_fy_topic').value)
        self.external_prediction_lead_samples = max(
            0, int(self.get_parameter('external_prediction_lead_samples').value)
        )
        self.external_max_queue_size = max(10, int(self.get_parameter('external_max_queue_size').value))
        self.log_interval = int(self.get_parameter('log_interval').value)
        self.require_on_road = bool(self.get_parameter('require_on_road').value)
        self.csv_output_path = str(self.get_parameter('csv_output_path').value)
        self.plot_output_dir = str(self.get_parameter('plot_output_dir').value)
        self.plot_max_points = int(self.get_parameter('plot_max_points').value)

        try:
            self.min_fz = float(self.get_parameter('min_fz_threshold').value)
        except ParameterUninitializedException:
            raise RuntimeError(
                "Parameter 'min_fz_threshold' must be set explicitly - there is no built-in "
                "default. A default sized for a full-size vehicle would silently reject every "
                "sample from a scaled car (e.g. the On-Track-SysID models in this repo are "
                "3.5-4.5kg, static per-wheel load ~8.6-11N). Pick a value well below your "
                "vehicle's static per-wheel load (~mass*9.81/4 N) so it only filters "
                "near-zero/airborne-wheel samples - set it via "
                "config/benchmark_config.yaml or 'min_fz_threshold:=<value>' on the launch line."
            )

        valid_modes = {'internal_pacejka', 'external_topic'}
        if self.benchmark_mode not in valid_modes:
            self.get_logger().debug(
                f"Invalid benchmark_mode='{self.benchmark_mode}'. Falling back to 'internal_pacejka'."
            )
            self.benchmark_mode = 'internal_pacejka'

        self.c_pf = self._get_optional_double_array('c_pf')
        self.c_pr = self._get_optional_double_array('c_pr')
        self.have_identified_params = self.c_pf is not None and self.c_pr is not None
        self.m = float(self.get_parameter('m').value)
        self.I_z = float(self.get_parameter('I_z').value)
        self.l_f = float(self.get_parameter('l_f').value)
        self.l_r = float(self.get_parameter('l_r').value)
        self.l_wb = float(self.get_parameter('l_wb').value)

        model_file = str(self.get_parameter('model_file').value)
        if model_file != '':
            self._load_model_if_available(model_file)

        self.nominal_c_pf = None
        self.nominal_c_pr = None
        nominal_model_file = str(self.get_parameter('nominal_model_file').value)
        if nominal_model_file != '':
            self._load_nominal_model_if_available(nominal_model_file)

        self._logged_waiting_for_params = False

        self.enable_state_benchmarking = bool(self.get_parameter('enable_state_benchmarking').value)
        if self.enable_state_benchmarking:
            if self.m <= 0.0 or self.I_z <= 0.0 or self.l_f <= 0.0 or self.l_r <= 0.0 or self.l_wb <= 0.0:
                self.get_logger().debug(
                    'enable_state_benchmarking requested but vehicle model constants '
                    '(m, I_z, l_f, l_r, l_wb) are missing/zero. Provide them via model_file '
                    '(the On-Track-SysID *_pacejka.txt format already contains them) or as '
                    'explicit parameters. Disabling state benchmarking.'
                )
                self.enable_state_benchmarking = False
            else:
                self.F_zf = self.m * G * self.l_r / self.l_wb
                self.F_zr = self.m * G * self.l_f / self.l_wb

        self.metrics = {
            'fl_fy': OnlineBenchmark('FL Fy'),
            'fr_fy': OnlineBenchmark('FR Fy'),
            'rl_fy': OnlineBenchmark('RL Fy'),
            'rr_fy': OnlineBenchmark('RR Fy'),
            'front_sum_fy': OnlineBenchmark('Front axle Fy sum'),
            'rear_sum_fy': OnlineBenchmark('Rear axle Fy sum'),
            'total_sum_fy': OnlineBenchmark('Vehicle total Fy sum'),
            'v_y': OnlineBenchmark('v_y (lateral velocity)'),
            'omega': OnlineBenchmark('omega (yaw rate)'),
        }
        self.history = {
            key: HistoryBuffer(self.plot_max_points) for key, _, _ in FORCE_SIGNALS + STATE_SIGNALS
        }
        # Per-axle (slip_angle, Fy_ground_truth, Fy_model) triples for the
        # Pacejka curve validation plot (internal_pacejka mode only - see
        # _plot_pacejka_curve). Reuses HistoryBuffer's bounded-memory
        # decimation with 'stamps' repurposed to hold slip angle instead of
        # a timestamp.
        self.slip_history = {
            'front': HistoryBuffer(self.plot_max_points),
            'rear': HistoryBuffer(self.plot_max_points),
        }
        self.sample_count = 0
        self.state_sample_count = 0
        self.latest_gt = None
        self.gt_queue = deque()
        self.est_queue = deque()
        self.drop_count_gt = 0
        self.drop_count_est = 0

        self.current_delta = 0.0
        self.prev_v_y = 0.0
        self.prev_omega = 0.0
        self.last_odom_time = None

        self.summary_pub = self.create_publisher(String, '/benchmarking/tire_force_summary', 10)

        if self.enable_force_benchmarking and not _HELLOCM_MSGS_AVAILABLE:
            self.get_logger().debug(
                'enable_force_benchmarking requested but hellocm_msgs (IPG CarMaker\'s tire '
                'forces message package) is not available in this workspace - disabling Fy '
                'benchmarking. State benchmarking (v_y, omega) is unaffected.'
            )
            self.enable_force_benchmarking = False

        self.estimation_pub = None
        self.sub = None
        self.ext_sub = None
        if self.enable_force_benchmarking:
            self.estimation_pub = self.create_publisher(
                Float64MultiArray, '/benchmarking/tire_force_fy_estimate', 10
            )
            self.sub = self.create_subscription(
                TireForcesArray,
                tire_forces_topic,
                self.tire_forces_callback,
                10,
            )
            if self.benchmark_mode == 'external_topic':
                self.ext_sub = self.create_subscription(
                    Float64MultiArray,
                    self.estimated_fy_topic,
                    self.estimated_fy_callback,
                    10,
                )

        self.odom_sub = None
        self.ackermann_sub = None
        self.state_estimate_pub = None
        self.state_sensor_pub = None
        self.state_error_pub = None
        if self.enable_state_benchmarking:
            odom_topic = str(self.get_parameter('odom_topic').value)
            ackermann_topic = str(self.get_parameter('ackermann_cmd_topic').value)
            state_estimate_topic = str(self.get_parameter('state_estimate_topic').value)
            state_sensor_topic = str(self.get_parameter('state_sensor_topic').value)
            state_error_topic = str(self.get_parameter('state_error_topic').value)

            self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
            self.ackermann_sub = self.create_subscription(
                AckermannDriveStamped, ackermann_topic, self.ackermann_callback, 10
            )
            self.state_estimate_pub = self.create_publisher(Float64MultiArray, state_estimate_topic, 10)
            self.state_sensor_pub = self.create_publisher(Float64MultiArray, state_sensor_topic, 10)
            self.state_error_pub = self.create_publisher(Float64MultiArray, state_error_topic, 10)

        identified_params_service = str(self.get_parameter('identified_params_service').value)
        self.identified_params_srv = self.create_service(
            IdentifiedParam, identified_params_service, self.handle_identified_params
        )

        self.csv_file = None
        self.csv_writer = None
        self.state_csv_file = None
        self.state_csv_writer = None
        self._setup_csv_if_enabled()

        # Startup banner - debug-only: this node should be silent except when
        # handle_identified_params actually receives a new identification.
        # Recoverable with --log-level debug.
        self.get_logger().debug('TireForceBenchmarkNode started')
        self.get_logger().debug(f'Force (Fy) benchmarking enabled: {self.enable_force_benchmarking}')
        if self.enable_force_benchmarking:
            self.get_logger().debug(f'Benchmark mode: {self.benchmark_mode}')
            self.get_logger().debug(f'Subscribing to: {tire_forces_topic}')
            if self.benchmark_mode == 'external_topic':
                self.get_logger().debug(f'Subscribing estimated Fy to: {self.estimated_fy_topic}')
                self.get_logger().debug(
                    f'External alignment lead samples: {self.external_prediction_lead_samples}'
                )
                self.get_logger().debug(f'External alignment max queue size: {self.external_max_queue_size}')
        self.get_logger().debug(f'Identified-params service: {identified_params_service}')
        if self.have_identified_params:
            self.get_logger().debug(f'Startup Pacejka params - C_Pf: {self.c_pf}, C_Pr: {self.c_pr}')
        else:
            self.get_logger().debug(
                'No c_pf/c_pr or model_file at startup - waiting for identified params via service '
                'before internal_pacejka benchmarking or state-prediction begins.'
            )
        self.get_logger().debug(f'State benchmarking enabled: {self.enable_state_benchmarking}')

    def _get_optional_double_array(self, name):
        try:
            return [float(v) for v in self.get_parameter(name).value]
        except ParameterUninitializedException:
            return None

    def handle_identified_params(self, request, response):
        if len(request.param_values) != 8:
            self.get_logger().warn(
                f'identified_params_service: expected 8 param_values, got {len(request.param_values)}'
            )
            response.ack = False
            return response

        values = [float(v) for v in request.param_values]
        self.c_pf = values[0:4]
        self.c_pr = values[4:8]
        newly_active = not self.have_identified_params
        self.have_identified_params = True
        if newly_active:
            self.get_logger().info(
                f'Received identified Pacejka params via service - benchmarking now active. '
                f'C_Pf={self.c_pf} C_Pr={self.c_pr}'
            )
        else:
            self.get_logger().info(f'Updated Pacejka params via service: C_Pf={self.c_pf} C_Pr={self.c_pr}')
        response.ack = True
        return response

    def _load_model_if_available(self, model_file: str):
        model_path = Path(model_file)
        if not model_path.exists():
            self.get_logger().debug(
                f'model_file not found: {model_file}. Waiting for identified_params_service instead.'
            )
            return

        try:
            with model_path.open('r', encoding='utf-8') as f:
                model_data = yaml.safe_load(f)
            c_pf = model_data.get('C_Pf', None)
            c_pr = model_data.get('C_Pr', None)
            if c_pf is not None and len(c_pf) == 4:
                self.c_pf = [float(v) for v in c_pf]
            if c_pr is not None and len(c_pr) == 4:
                self.c_pr = [float(v) for v in c_pr]
            if self.c_pf is not None and self.c_pr is not None:
                self.have_identified_params = True

            for key in ('m', 'I_z', 'l_f', 'l_r', 'l_wb'):
                value = model_data.get(key, None)
                if value is not None:
                    setattr(self, key, float(value))

            self.get_logger().debug(f'Loaded vehicle model from model_file: {model_file}')
        except Exception as exc:
            self.get_logger().debug(
                f'Failed to load model_file {model_file}: {exc}. Waiting for identified_params_service instead.'
            )

    def _load_nominal_model_if_available(self, nominal_model_file: str):
        # Deliberately separate from _load_model_if_available: only ever sets
        # nominal_c_pf/nominal_c_pr (for _plot_identified_vs_nominal), never
        # c_pf/c_pr/have_identified_params - a nominal reference model here
        # must not seed or contaminate the live benchmark.
        model_path = Path(nominal_model_file)
        if not model_path.exists():
            self.get_logger().debug(f'nominal_model_file not found: {nominal_model_file}.')
            return

        try:
            with model_path.open('r', encoding='utf-8') as f:
                model_data = yaml.safe_load(f)
            # Flat format (On-Track-SysID's models/<car>/<car>_pacejka.txt): top-level
            # C_Pf/C_Pr. Falls back to On-Track-SysID's own params/pacejka_params.yaml
            # nesting (pacejka_ref.C_Pf_ref/C_Pr_ref) - the "nominal" half of the same
            # model/ref pair its own plot_results.py compares the identified fit against.
            c_pf = model_data.get('C_Pf', None)
            c_pr = model_data.get('C_Pr', None)
            if c_pf is None or c_pr is None:
                pacejka_ref = model_data.get('pacejka_ref', {})
                c_pf = c_pf if c_pf is not None else pacejka_ref.get('C_Pf_ref', None)
                c_pr = c_pr if c_pr is not None else pacejka_ref.get('C_Pr_ref', None)
            if c_pf is not None and len(c_pf) == 4:
                self.nominal_c_pf = [float(v) for v in c_pf]
            if c_pr is not None and len(c_pr) == 4:
                self.nominal_c_pr = [float(v) for v in c_pr]
            self.get_logger().debug(f'Loaded nominal model from nominal_model_file: {nominal_model_file}')
        except Exception as exc:
            self.get_logger().debug(f'Failed to load nominal_model_file {nominal_model_file}: {exc}.')

    def _setup_csv_if_enabled(self):
        if self.csv_output_path == '':
            return

        output_path = Path(self.csv_output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec',
            'fl_fy_gt', 'fl_fy_est', 'fr_fy_gt', 'fr_fy_est',
            'rl_fy_gt', 'rl_fy_est', 'rr_fy_gt', 'rr_fy_est',
            'front_sum_gt', 'front_sum_est',
            'rear_sum_gt', 'rear_sum_est',
            'total_sum_gt', 'total_sum_est',
        ])
        self.get_logger().debug(f'CSV logging enabled: {self.csv_output_path}')

        if self.enable_state_benchmarking:
            state_csv_path = output_path.with_name(f'{output_path.stem}_states{output_path.suffix}')
            self.state_csv_file = state_csv_path.open('w', newline='', encoding='utf-8')
            self.state_csv_writer = csv.writer(self.state_csv_file)
            self.state_csv_writer.writerow(['stamp_sec', 'v_y_gt', 'v_y_est', 'omega_gt', 'omega_est'])
            self.get_logger().debug(f'State CSV logging enabled: {state_csv_path}')

    def _use_sample(self, tire_msg) -> bool:
        if self.require_on_road and not tire_msg.on_road:
            return False
        if abs(tire_msg.fz) < self.min_fz:
            return False
        if math.isnan(tire_msg.slip_angle) or math.isnan(tire_msg.fz):
            return False
        return True

    def tire_forces_callback(self, msg: TireForcesArray):
        if not self.enable_force_benchmarking:
            return
        fl = msg.front_left
        fr = msg.front_right
        rl = msg.rear_left
        rr = msg.rear_right

        tires = [fl, fr, rl, rr]
        if not all(self._use_sample(t) for t in tires):
            return

        front_gt = fl.fy + fr.fy
        rear_gt = rl.fy + rr.fy
        total_gt = front_gt + rear_gt
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.benchmark_mode == 'internal_pacejka':
            if not self.have_identified_params:
                if not self._logged_waiting_for_params:
                    self.get_logger().debug(
                        'internal_pacejka: no identified Pacejka params yet - waiting for '
                        'identified_params_service before benchmarking starts.'
                    )
                    self._logged_waiting_for_params = True
                return
            fl_est = float(pacejka_formula(self.c_pf, fl.slip_angle, fl.fz))
            fr_est = float(pacejka_formula(self.c_pf, fr.slip_angle, fr.fz))
            rl_est = float(pacejka_formula(self.c_pr, rl.slip_angle, rl.fz))
            rr_est = float(pacejka_formula(self.c_pr, rr.slip_angle, rr.fz))
            if self.plot_output_dir:
                self.slip_history['front'].add(fl.slip_angle, fl.fy, fl_est)
                self.slip_history['front'].add(fr.slip_angle, fr.fy, fr_est)
                self.slip_history['rear'].add(rl.slip_angle, rl.fy, rl_est)
                self.slip_history['rear'].add(rr.slip_angle, rr.fy, rr_est)
            self._benchmark_and_publish(
                stamp_sec,
                [fl.fy, fr.fy, rl.fy, rr.fy],
                [fl_est, fr_est, rl_est, rr_est],
                publish_estimate=True,
            )
            return

        self.latest_gt = {
            'stamp_sec': stamp_sec,
            'fy': [fl.fy, fr.fy, rl.fy, rr.fy],
            'front_gt': front_gt,
            'rear_gt': rear_gt,
            'total_gt': total_gt,
        }
        self.gt_queue.append(self.latest_gt)
        self._trim_queues_if_needed()
        self._try_external_queue_alignment()

    def estimated_fy_callback(self, msg: Float64MultiArray):
        if not self.enable_force_benchmarking or self.benchmark_mode != 'external_topic':
            return
        if len(msg.data) < 4:
            self.get_logger().debug('estimated_fy_topic message must contain at least 4 values [FL, FR, RL, RR].')
            return

        fl_est, fr_est, rl_est, rr_est = [float(v) for v in msg.data[:4]]
        if any(math.isnan(v) for v in [fl_est, fr_est, rl_est, rr_est]):
            return
        self.est_queue.append([fl_est, fr_est, rl_est, rr_est])
        self._trim_queues_if_needed()
        self._try_external_queue_alignment()

    def _trim_queues_if_needed(self):
        while len(self.gt_queue) > self.external_max_queue_size:
            self.gt_queue.popleft()
            self.drop_count_gt += 1

        while len(self.est_queue) > self.external_max_queue_size:
            self.est_queue.popleft()
            self.drop_count_est += 1

    def _try_external_queue_alignment(self):
        if self.benchmark_mode != 'external_topic':
            return

        required_gt = self.external_prediction_lead_samples + 1
        paired = 0

        while len(self.gt_queue) >= required_gt and len(self.est_queue) > 0:
            gt = self.gt_queue[self.external_prediction_lead_samples]
            est = self.est_queue.popleft()

            self._benchmark_and_publish(
                gt['stamp_sec'],
                gt['fy'],
                est,
                publish_estimate=False,
            )

            # Slide the GT window forward by exactly one sample per estimate
            # consumed - popping `required_gt` here (as a previous version of
            # this code did) discards `lead` extra GT samples every
            # iteration, drifting the estimate[k]<->ground_truth[k+lead]
            # correspondence documented in the README further apart each
            # time (only accidentally correct for lead=0).
            self.gt_queue.popleft()
            paired += 1

        if paired > 0 and (self.sample_count % self.log_interval == 0):
            if self.drop_count_gt > 0 or self.drop_count_est > 0:
                self.get_logger().debug(
                    f'Queue drops detected: gt={self.drop_count_gt}, est={self.drop_count_est}. '
                    f'Consider increasing external_max_queue_size or adjusting external_prediction_lead_samples.'
                )

    def _build_summary_lines(self):
        keys = [key for key, _, _ in FORCE_SIGNALS + STATE_SIGNALS]
        return [self.metrics[key].summary() for key in keys]

    def _log_and_publish_summary(self):
        summary_text = '\n'.join(self._build_summary_lines())
        self.get_logger().debug(f'\n{summary_text}')

        summary_msg = String()
        summary_msg.data = summary_text
        self.summary_pub.publish(summary_msg)

    def _benchmark_and_publish(self, stamp_sec: float, fy_gt, fy_est, publish_estimate: bool):
        fl_gt, fr_gt, rl_gt, rr_gt = fy_gt
        fl_est, fr_est, rl_est, rr_est = fy_est

        self.metrics['fl_fy'].update(fl_gt, fl_est)
        self.metrics['fr_fy'].update(fr_gt, fr_est)
        self.metrics['rl_fy'].update(rl_gt, rl_est)
        self.metrics['rr_fy'].update(rr_gt, rr_est)

        front_gt = fl_gt + fr_gt
        rear_gt = rl_gt + rr_gt
        total_gt = front_gt + rear_gt
        front_est = fl_est + fr_est
        rear_est = rl_est + rr_est
        total_est = front_est + rear_est

        self.metrics['front_sum_fy'].update(front_gt, front_est)
        self.metrics['rear_sum_fy'].update(rear_gt, rear_est)
        self.metrics['total_sum_fy'].update(total_gt, total_est)

        if self.plot_output_dir:
            self.history['fl_fy'].add(stamp_sec, fl_gt, fl_est)
            self.history['fr_fy'].add(stamp_sec, fr_gt, fr_est)
            self.history['rl_fy'].add(stamp_sec, rl_gt, rl_est)
            self.history['rr_fy'].add(stamp_sec, rr_gt, rr_est)
            self.history['front_sum_fy'].add(stamp_sec, front_gt, front_est)
            self.history['rear_sum_fy'].add(stamp_sec, rear_gt, rear_est)
            self.history['total_sum_fy'].add(stamp_sec, total_gt, total_est)

        if publish_estimate:
            est_msg = Float64MultiArray()
            est_msg.data = [fl_est, fr_est, rl_est, rr_est, front_est, rear_est, total_est]
            self.estimation_pub.publish(est_msg)

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                stamp_sec,
                fl_gt, fl_est, fr_gt, fr_est,
                rl_gt, rl_est, rr_gt, rr_est,
                front_gt, front_est,
                rear_gt, rear_est,
                total_gt, total_est,
            ])

        self.sample_count += 1
        if self.sample_count % self.log_interval == 0:
            self._log_and_publish_summary()

    def odom_callback(self, msg: Odometry):
        if not self.enable_state_benchmarking:
            return

        v_x = msg.twist.twist.linear.x
        v_y_real = msg.twist.twist.linear.y
        omega_real = msg.twist.twist.angular.z
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        dt = None
        if self.last_odom_time is not None:
            dt = (stamp - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = stamp

        # Same guards as On-Track-SysID's on_track_sys_id.py:publish_estimates -
        # first sample / non-monotonic stamp (dt is None or <=0), a timing gap
        # (dt too large to trust a one-step Euler prediction), or the car
        # being effectively stopped (slip-angle formula divides by v_x).
        if dt is None or dt <= 1e-5 or dt > 0.2 or v_x < 0.1 or not self.have_identified_params:
            self.prev_v_y = v_y_real
            self.prev_omega = omega_real
            return

        delta = self.current_delta

        alpha_f = -np.arctan((self.prev_v_y + self.prev_omega * self.l_f) / v_x) + delta
        alpha_r = -np.arctan((self.prev_v_y - self.prev_omega * self.l_r) / v_x)

        f_f = float(pacejka_formula(self.c_pf, alpha_f, self.F_zf))
        f_r = float(pacejka_formula(self.c_pr, alpha_r, self.F_zr))

        v_y_dot = (1.0 / self.m) * (f_r + f_f * math.cos(delta) - self.m * v_x * self.prev_omega)
        omega_dot = (1.0 / self.I_z) * (f_f * self.l_f * math.cos(delta) - f_r * self.l_r)

        v_y_pred = self.prev_v_y + v_y_dot * dt
        omega_pred = self.prev_omega + omega_dot * dt

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._benchmark_state(stamp_sec, v_x, v_y_real, omega_real, delta, v_y_pred, omega_pred)

        self.prev_v_y = v_y_real
        self.prev_omega = omega_real

    def ackermann_callback(self, msg: AckermannDriveStamped):
        self.current_delta = msg.drive.steering_angle

    def _benchmark_state(self, stamp_sec, v_x, v_y_real, omega_real, delta, v_y_pred, omega_pred):
        self.metrics['v_y'].update(v_y_real, v_y_pred)
        self.metrics['omega'].update(omega_real, omega_pred)

        if self.plot_output_dir:
            self.history['v_y'].add(stamp_sec, v_y_real, v_y_pred)
            self.history['omega'].add(stamp_sec, omega_real, omega_pred)

        sensor_msg = Float64MultiArray()
        sensor_msg.data = [v_x, v_y_real, omega_real, delta]
        self.state_sensor_pub.publish(sensor_msg)

        estimate_msg = Float64MultiArray()
        estimate_msg.data = [v_x, v_y_pred, omega_pred]
        self.state_estimate_pub.publish(estimate_msg)

        error_msg = Float64MultiArray()
        error_msg.data = [abs(v_y_real - v_y_pred), abs(omega_real - omega_pred)]
        self.state_error_pub.publish(error_msg)

        if self.state_csv_writer is not None:
            self.state_csv_writer.writerow([stamp_sec, v_y_real, v_y_pred, omega_real, omega_pred])

        self.state_sample_count += 1
        if self.state_sample_count % self.log_interval == 0:
            self._log_and_publish_summary()

    def _export_plots(self):
        if not self.plot_output_dir:
            return

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().debug('matplotlib not available - skipping benchmark plot export.')
            return

        self._apply_academic_style(plt)

        out_dir = Path(self.plot_output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        self._plot_timeseries_grid(
            plt, FORCE_SIGNALS, out_dir / 'tire_forces_timeseries.png',
            'Tire Lateral Force: Ground Truth vs. Estimate',
        )
        self._plot_timeseries_grid(
            plt, STATE_SIGNALS, out_dir / 'vehicle_states_timeseries.png',
            'Vehicle States: Ground Truth vs. Estimate',
        )
        self._plot_error_hist_grid(
            plt, FORCE_SIGNALS, out_dir / 'tire_forces_error_hist.png',
            'Tire Lateral Force Error Distribution',
        )
        self._plot_error_hist_grid(
            plt, STATE_SIGNALS, out_dir / 'vehicle_states_error_hist.png',
            'Vehicle State Error Distribution',
        )
        self._plot_parity_grid(
            plt, FORCE_SIGNALS, out_dir / 'tire_forces_parity.png',
            'Tire Lateral Force: Estimate vs. Ground Truth (Parity)',
        )
        self._plot_parity_grid(
            plt, STATE_SIGNALS, out_dir / 'vehicle_states_parity.png',
            'Vehicle State: Estimate vs. Ground Truth (Parity)',
        )
        self._plot_pacejka_curve(plt, out_dir / 'pacejka_curve_validation.png')
        self._plot_identified_vs_nominal(plt, out_dir / 'pacejka_identified_vs_nominal.png')
        self._plot_metrics_table(plt, FORCE_SIGNALS + STATE_SIGNALS, out_dir / 'metrics_summary.png')

        self.get_logger().debug(f'Benchmark plots saved to: {out_dir}')

    def _apply_academic_style(self, plt):
        # Serif, gridded, high-DPI - the conventions of the vehicle-dynamics
        # identification literature this package's model traces back to (see
        # docs/tire_force_benchmark.md References: Pacejka/Bakker, Dikici et
        # al. 2024), rather than a UI/dashboard look.
        plt.rcParams.update({
            'font.family': 'serif',
            'font.size': 10,
            'axes.titlesize': 11,
            'axes.labelsize': 10,
            'legend.fontsize': 8,
            'axes.grid': True,
            'grid.color': COLOR_GRID,
            'grid.alpha': 0.6,
            'grid.linewidth': 0.6,
            'axes.edgecolor': COLOR_MUTED,
            'axes.labelcolor': COLOR_INK,
            'text.color': COLOR_INK,
            'xtick.color': COLOR_SECONDARY_INK,
            'ytick.color': COLOR_SECONDARY_INK,
            'axes.spines.top': False,
            'axes.spines.right': False,
            'legend.frameon': True,
            'legend.framealpha': 0.9,
            'legend.edgecolor': COLOR_GRID,
            'figure.dpi': 120,
            'savefig.dpi': 300,
        })

    def _grid_shape(self, n):
        ncols = 1 if n <= 2 else 2
        nrows = math.ceil(n / ncols)
        return nrows, ncols

    def _plot_timeseries_grid(self, plt, signals, save_path, suptitle):
        nrows, ncols = self._grid_shape(len(signals))
        fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 3 * nrows), squeeze=False)

        for idx, (key, label, unit) in enumerate(signals):
            ax = axes[idx // ncols][idx % ncols]
            hist = self.history[key]
            if len(hist.stamps) == 0:
                ax.set_title(f'{label} (no data)')
                ax.axis('off')
                continue
            t0 = hist.stamps[0]
            t = [s - t0 for s in hist.stamps]
            ax.plot(t, hist.gt, label='Ground truth', linewidth=1.2, color=COLOR_GT)
            ax.plot(t, hist.est, label='Estimate', linewidth=1.2, linestyle='--', color=COLOR_EST)
            ax.set_title(label)
            ax.set_xlabel('Time [s]')
            ax.set_ylabel(f'{label} [{unit}]')
            ax.legend(loc='best')

        for idx in range(len(signals), nrows * ncols):
            axes[idx // ncols][idx % ncols].axis('off')

        fig.suptitle(suptitle, fontsize=14)
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_error_hist_grid(self, plt, signals, save_path, suptitle):
        nrows, ncols = self._grid_shape(len(signals))
        fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 3 * nrows), squeeze=False)

        for idx, (key, label, unit) in enumerate(signals):
            ax = axes[idx // ncols][idx % ncols]
            hist = self.history[key]
            if len(hist.stamps) == 0:
                ax.set_title(f'{label} (no data)')
                ax.axis('off')
                continue
            errors = [g - e for g, e in zip(hist.gt, hist.est)]
            mean_err = float(np.mean(errors))
            ax.hist(errors, bins=40, color=COLOR_GT, alpha=0.75,
                    edgecolor='white', linewidth=0.3, label='Error samples')
            ax.axvline(0.0, color=COLOR_MUTED, linestyle=':', linewidth=1.2, label='Zero error')
            ax.axvline(mean_err, color=COLOR_EST, linestyle='--', linewidth=1.5,
                       label=f'Mean bias = {mean_err:.3f}')
            ax.set_title(label)
            ax.set_xlabel(f'Error (GT - Estimate) [{unit}]')
            ax.set_ylabel('Count')
            ax.legend(loc='best')

        for idx in range(len(signals), nrows * ncols):
            axes[idx // ncols][idx % ncols].axis('off')

        fig.suptitle(suptitle, fontsize=14)
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_parity_grid(self, plt, signals, save_path, suptitle):
        # Predicted-vs-measured "parity" scatter with a y=x reference line -
        # the standard model-validation figure in the tire/vehicle-state
        # identification literature (e.g. Fig. 5/6 style validation in
        # Dikici et al. 2024, arXiv:2411.17508), complementing the
        # time-series overlay with a single view of fit quality across the
        # whole run.
        nrows, ncols = self._grid_shape(len(signals))
        fig, axes = plt.subplots(nrows, ncols, figsize=(4.5 * ncols, 4.3 * nrows), squeeze=False)

        for idx, (key, label, unit) in enumerate(signals):
            ax = axes[idx // ncols][idx % ncols]
            hist = self.history[key]
            if len(hist.stamps) == 0:
                ax.set_title(f'{label} (no data)')
                ax.axis('off')
                continue

            gt = np.asarray(hist.gt)
            est = np.asarray(hist.est)
            ax.scatter(gt, est, s=10, alpha=0.35, color=COLOR_GT,
                      edgecolors='none', label='Samples')

            lo = float(min(gt.min(), est.min()))
            hi = float(max(gt.max(), est.max()))
            pad = 0.05 * (hi - lo) if hi > lo else 1.0
            bounds = [lo - pad, hi + pad]
            ax.plot(bounds, bounds, color=COLOR_EST, linestyle='--',
                   linewidth=1.5, label='Ideal (y = x)')

            m = self.metrics[key].metrics()
            r2 = f"{m['r_squared']:.3f}" if not math.isnan(m['r_squared']) else 'N/A'
            ax.text(
                0.05, 0.95, f"RMSE = {m['rmse']:.3f} {unit}\n$R^2$ = {r2}",
                transform=ax.transAxes, va='top', ha='left', fontsize=8,
                bbox=dict(boxstyle='round', facecolor='white', edgecolor=COLOR_GRID, alpha=0.9),
            )

            ax.set_xlim(bounds)
            ax.set_ylim(bounds)
            ax.set_aspect('equal', adjustable='box')
            ax.set_title(label)
            ax.set_xlabel(f'Ground truth {label} [{unit}]')
            ax.set_ylabel(f'Estimate {label} [{unit}]')
            ax.legend(loc='lower right')

        for idx in range(len(signals), nrows * ncols):
            axes[idx // ncols][idx % ncols].axis('off')

        fig.suptitle(suptitle, fontsize=14)
        # set_aspect('equal') above makes tight_layout's automatic top margin
        # unreliable (rows expand to stay square, crowding the suptitle) -
        # reserve the margin explicitly instead.
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_pacejka_curve(self, plt, save_path):
        # The other canonical figure in this literature (Pacejka & Bakker
        # 1992; Fig. 6-style validation in Dikici et al. 2024): measured Fy
        # vs. slip angle scatter overlaid with the identified Magic Formula
        # curve, per axle. Only meaningful in internal_pacejka mode (the
        # mode that actually holds a live C_Pf/C_Pr model) and once a model
        # has been identified.
        if self.benchmark_mode != 'internal_pacejka' or not self.have_identified_params:
            return

        if self.m <= 0.0 or self.l_wb <= 0.0:
            self.get_logger().debug(
                'Skipping pacejka_curve_validation.png - vehicle mass/wheelbase constants '
                '(m, l_f, l_r, l_wb) are required to compute the nominal static axle loads '
                'the fitted curve is drawn at.'
            )
            return

        fzf = self.m * G * self.l_r / self.l_wb
        fzr = self.m * G * self.l_f / self.l_wb
        axle_specs = [
            ('front', 'Front axle', self.c_pf, fzf),
            ('rear', 'Rear axle', self.c_pr, fzr),
        ]

        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
        any_data = False
        for ax, (key, label, params, fz) in zip(axes, axle_specs):
            hist = self.slip_history[key]
            if len(hist.stamps) == 0:
                ax.set_title(f'{label} (no data)')
                ax.axis('off')
                continue
            any_data = True

            alpha = np.asarray(hist.stamps)
            fy_gt = np.asarray(hist.gt)
            ax.scatter(np.degrees(alpha), fy_gt, s=8, alpha=0.3, color=COLOR_GT,
                      edgecolors='none', label='Measured (ground truth)')

            alpha_sweep = np.linspace(alpha.min(), alpha.max(), 200)
            fy_model = pacejka_formula(params, alpha_sweep, fz)
            ax.plot(np.degrees(alpha_sweep), fy_model, color=COLOR_EST,
                   linewidth=2.0, label='Identified Magic Formula model')

            ax.set_title(f'{label} ($F_z \\approx$ {fz:.1f} N)')
            ax.set_xlabel(r'Slip angle $\alpha$ [deg]')
            ax.set_ylabel(r'Lateral force $F_y$ [N]')
            ax.legend(loc='best')

        if not any_data:
            plt.close(fig)
            return

        fig.suptitle('Pacejka Magic Formula Validation: $F_y$ vs. Slip Angle', fontsize=14)
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_identified_vs_nominal(self, plt, save_path):
        # Model-vs-model comparison (no data scatter): the freshly identified
        # Magic Formula against a nominal/prior reference model
        # (nominal_model_file - see _load_nominal_model_if_available), swept
        # analytically over a fixed slip-angle range at the nominal static
        # axle load. Complements _plot_pacejka_curve (identified model vs.
        # measured data) with "how much did identification actually change
        # the model" - the other comparison this literature reports (Pacejka
        # & Bakker 1992; Dikici et al. 2024).
        if not self.have_identified_params or self.nominal_c_pf is None or self.nominal_c_pr is None:
            return

        if self.m <= 0.0 or self.l_wb <= 0.0:
            self.get_logger().debug(
                'Skipping pacejka_identified_vs_nominal.png - vehicle mass/wheelbase constants '
                '(m, l_f, l_r, l_wb) are required to compute the nominal static axle loads.'
            )
            return

        fzf = self.m * G * self.l_r / self.l_wb
        fzr = self.m * G * self.l_f / self.l_wb
        alpha_sweep = np.linspace(-0.20, 0.20, 200)
        axle_specs = [
            ('Front Tires', self.c_pf, self.nominal_c_pf, fzf, r'$F_{yf}$ [N]', r'$\alpha_f$ [rad]'),
            ('Rear Tires', self.c_pr, self.nominal_c_pr, fzr, r'$F_{yr}$ [N]', r'$\alpha_r$ [rad]'),
        ]

        fig, axes = plt.subplots(2, 1, figsize=(7, 7))
        for ax, (title, identified_params, nominal_params, fz, ylabel, xlabel) in zip(axes, axle_specs):
            fy_identified = pacejka_formula(identified_params, alpha_sweep, fz)
            fy_nominal = pacejka_formula(nominal_params, alpha_sweep, fz)
            ax.plot(alpha_sweep, fy_identified, color=COLOR_EST, linewidth=2.0, label='Identified Model')
            ax.plot(alpha_sweep, fy_nominal, color=COLOR_NOMINAL, linewidth=2.0, label='Nominal Model')
            ax.set_title(title)
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.legend(loc='best')

        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_metrics_table(self, plt, signals, save_path):
        col_labels = ['Signal', 'N', 'RMSE', 'MAE', 'NRMSE', 'MaxAE', 'Bias', 'Std', 'R2']
        rows = []
        for key, label, _unit in signals:
            m = self.metrics[key].metrics()
            if m['n_samples'] == 0:
                rows.append([label, '0', '-', '-', '-', '-', '-', '-', '-'])
                continue
            nrmse = f"{m['nrmse']:.3f}" if not math.isnan(m['nrmse']) else 'N/A'
            r2 = f"{m['r_squared']:.3f}" if not math.isnan(m['r_squared']) else 'N/A'
            rows.append([
                label, str(m['n_samples']), f"{m['rmse']:.3f}", f"{m['mae']:.3f}",
                nrmse, f"{m['max_ae']:.3f}", f"{m['bias']:.3f}", f"{m['std_dev']:.3f}", r2,
            ])

        fig, ax = plt.subplots(figsize=(13, 0.4 * len(rows) + 1.2))
        ax.axis('off')
        table = ax.table(cellText=rows, colLabels=col_labels, loc='center', cellLoc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        # Columns default to equal width regardless of content - without this, long
        # labels ("Front axle Fy sum", "Vehicle total Fy sum") get clipped instead
        # of the column widening to fit them.
        table.auto_set_column_width(col=list(range(len(col_labels))))
        table.scale(1, 1.4)
        for (row, _col), cell in table.get_celld().items():
            if row == 0:
                cell.set_facecolor(COLOR_GRID)
                cell.set_text_props(weight='bold')
        fig.suptitle('Benchmark Metrics Summary', fontsize=14)
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        if self.state_csv_file is not None:
            self.state_csv_file.flush()
            self.state_csv_file.close()
        self._export_plots()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TireForceBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
