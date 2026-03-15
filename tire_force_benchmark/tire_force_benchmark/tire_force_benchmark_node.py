#!/usr/bin/env python3

import csv
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

from hellocm_msgs.msg import TireForcesArray

from tire_force_benchmark.online_metrics import OnlineBenchmark


def pacejka_formula(params, alpha, fz):
    b, c, d, e = params[0], params[1], params[2], params[3]
    return fz * d * np.sin(c * np.arctan(b * alpha - e * (b * alpha - np.arctan(b * alpha))))


class TireForceBenchmarkNode(Node):
    def __init__(self):
        super().__init__('tire_force_benchmark_node')

        self.declare_parameter('benchmark_mode', 'internal_pacejka')
        self.declare_parameter('tire_forces_topic', '/tire_forces')
        self.declare_parameter('estimated_fy_topic', '/estimated_tire_force_fy')
        self.declare_parameter('external_prediction_lead_samples', 1)
        self.declare_parameter('external_max_queue_size', 2000)
        self.declare_parameter('log_interval', 200)
        self.declare_parameter('min_fz_threshold', 50.0)
        self.declare_parameter('require_on_road', True)
        self.declare_parameter('model_file', '')
        self.declare_parameter('c_pf', [6.63, 1.1052, 0.4316, 0.5193])
        self.declare_parameter('c_pr', [7.8594, 1.5468, 0.3589, 0.5631])
        self.declare_parameter('csv_output_path', '')

        self.benchmark_mode = str(self.get_parameter('benchmark_mode').value).strip()
        tire_forces_topic = self.get_parameter('tire_forces_topic').value
        self.estimated_fy_topic = str(self.get_parameter('estimated_fy_topic').value)
        self.external_prediction_lead_samples = max(
            0, int(self.get_parameter('external_prediction_lead_samples').value)
        )
        self.external_max_queue_size = max(10, int(self.get_parameter('external_max_queue_size').value))
        self.log_interval = int(self.get_parameter('log_interval').value)
        self.min_fz = float(self.get_parameter('min_fz_threshold').value)
        self.require_on_road = bool(self.get_parameter('require_on_road').value)
        self.csv_output_path = str(self.get_parameter('csv_output_path').value)

        valid_modes = {'internal_pacejka', 'external_topic'}
        if self.benchmark_mode not in valid_modes:
            self.get_logger().warn(
                f"Invalid benchmark_mode='{self.benchmark_mode}'. Falling back to 'internal_pacejka'."
            )
            self.benchmark_mode = 'internal_pacejka'

        self.c_pf = [float(v) for v in self.get_parameter('c_pf').value]
        self.c_pr = [float(v) for v in self.get_parameter('c_pr').value]
        if self.benchmark_mode == 'internal_pacejka':
            self._load_model_if_available(str(self.get_parameter('model_file').value))

        self.metrics = {
            'fl_fy': OnlineBenchmark('FL Fy'),
            'fr_fy': OnlineBenchmark('FR Fy'),
            'rl_fy': OnlineBenchmark('RL Fy'),
            'rr_fy': OnlineBenchmark('RR Fy'),
            'front_sum_fy': OnlineBenchmark('Front axle Fy sum'),
            'rear_sum_fy': OnlineBenchmark('Rear axle Fy sum'),
            'total_sum_fy': OnlineBenchmark('Vehicle total Fy sum'),
        }
        self.sample_count = 0
        self.latest_gt = None
        self.gt_queue = deque()
        self.est_queue = deque()
        self.drop_count_gt = 0
        self.drop_count_est = 0

        self.estimation_pub = self.create_publisher(Float64MultiArray, '/benchmarking/tire_force_fy_estimate', 10)
        self.summary_pub = self.create_publisher(String, '/benchmarking/tire_force_summary', 10)

        self.sub = self.create_subscription(
            TireForcesArray,
            tire_forces_topic,
            self.tire_forces_callback,
            10,
        )
        self.ext_sub = None
        if self.benchmark_mode == 'external_topic':
            self.ext_sub = self.create_subscription(
                Float64MultiArray,
                self.estimated_fy_topic,
                self.estimated_fy_callback,
                10,
            )

        self.csv_file = None
        self.csv_writer = None
        self._setup_csv_if_enabled()

        self.get_logger().info('TireForceBenchmarkNode started')
        self.get_logger().info(f'Benchmark mode: {self.benchmark_mode}')
        self.get_logger().info(f'Subscribing to: {tire_forces_topic}')
        if self.benchmark_mode == 'external_topic':
            self.get_logger().info(f'Subscribing estimated Fy to: {self.estimated_fy_topic}')
            self.get_logger().info(
                f'External alignment lead samples: {self.external_prediction_lead_samples}'
            )
            self.get_logger().info(f'External alignment max queue size: {self.external_max_queue_size}')
        else:
            self.get_logger().info(f'Front Pacejka params C_Pf: {self.c_pf}')
            self.get_logger().info(f'Rear Pacejka params C_Pr: {self.c_pr}')

    def _load_model_if_available(self, model_file: str):
        if model_file == '':
            return

        model_path = Path(model_file)
        if not model_path.exists():
            self.get_logger().warn(f'model_file not found: {model_file}. Using parameters from c_pf/c_pr.')
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
            self.get_logger().info(f'Loaded Pacejka coefficients from model_file: {model_file}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to load model_file {model_file}: {exc}. Using c_pf/c_pr.')

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
        self.get_logger().info(f'CSV logging enabled: {self.csv_output_path}')

    def _use_sample(self, tire_msg) -> bool:
        if self.require_on_road and not tire_msg.on_road:
            return False
        if abs(tire_msg.fz) < self.min_fz:
            return False
        if math.isnan(tire_msg.slip_angle) or math.isnan(tire_msg.fz):
            return False
        return True

    def tire_forces_callback(self, msg: TireForcesArray):
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
            fl_est = float(pacejka_formula(self.c_pf, fl.slip_angle, fl.fz))
            fr_est = float(pacejka_formula(self.c_pf, fr.slip_angle, fr.fz))
            rl_est = float(pacejka_formula(self.c_pr, rl.slip_angle, rl.fz))
            rr_est = float(pacejka_formula(self.c_pr, rr.slip_angle, rr.fz))
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
        if self.benchmark_mode != 'external_topic':
            return
        if len(msg.data) < 4:
            self.get_logger().warn('estimated_fy_topic message must contain at least 4 values [FL, FR, RL, RR].')
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

            for _ in range(required_gt):
                self.gt_queue.popleft()
            paired += 1

        if paired > 0 and (self.sample_count % self.log_interval == 0):
            if self.drop_count_gt > 0 or self.drop_count_est > 0:
                self.get_logger().warn(
                    f'Queue drops detected: gt={self.drop_count_gt}, est={self.drop_count_est}. '
                    f'Consider increasing external_max_queue_size or adjusting external_prediction_lead_samples.'
                )

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
            lines = [
                self.metrics['fl_fy'].summary(),
                self.metrics['fr_fy'].summary(),
                self.metrics['rl_fy'].summary(),
                self.metrics['rr_fy'].summary(),
                self.metrics['front_sum_fy'].summary(),
                self.metrics['rear_sum_fy'].summary(),
                self.metrics['total_sum_fy'].summary(),
            ]
            summary_text = '\n'.join(lines)
            self.get_logger().info(f'\n{summary_text}')

            summary_msg = String()
            summary_msg.data = summary_text
            self.summary_pub.publish(summary_msg)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
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
