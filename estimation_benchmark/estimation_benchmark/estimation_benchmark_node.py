#!/usr/bin/env python3
"""
Estimation Benchmarking Node for Vehicle Dynamics.

Performs configurable one-step and multi-step state prediction benchmarking
using the Pacejka-based dynamic single-track model identified by On-Track-SysID.

Paper reference: 2411.17508v1 — Learning-Based On-Track System Identification
for Scaled Autonomous Racing in Under a Minute.

Lifecycle:
    1. Init: loads config, subscribes to /odom, /drive, /sysid/training_complete.
    2. Waiting: buffers data but does NOT benchmark until training completes.
    3. Active: runs prediction + benchmarking with academic metrics.
"""

import csv
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, String

from estimation_benchmark.online_metrics import OnlineBenchmark
from estimation_benchmark.multi_step_predictor import (
    predict_multi_step,
    pacejka_formula,
)


class EstimationBenchmarkNode(Node):
    """ROS2 node for multi-step estimation benchmarking."""

    def __init__(self):
        super().__init__('estimation_benchmark_node')

        # ── Declare all parameters with defaults ──
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ackermann_cmd_topic', '/drive')
        self.declare_parameter('training_complete_topic', '/sysid/training_complete')
        self.declare_parameter('prediction_steps', [1, 5, 10])
        self.declare_parameter('dt', 0.02)
        self.declare_parameter('mass', 3.47)
        self.declare_parameter('I_z', 0.04712)
        self.declare_parameter('l_f', 0.15875)
        self.declare_parameter('l_r', 0.17145)
        self.declare_parameter('l_wb', 0.3302)
        self.declare_parameter('C_Pf', [7.17, 1.56, 0.69, 0.53])
        self.declare_parameter('C_Pr', [8.29, 2.11, 0.68, 0.40])
        self.declare_parameter('enable_tire_force_benchmark', False)
        self.declare_parameter('tire_forces_topic', '/tire_forces')
        self.declare_parameter('min_velocity', 0.5)
        self.declare_parameter('log_interval', 100)
        self.declare_parameter('csv_output_path', '')
        self.declare_parameter('data_buffer_size', 200)
        self.declare_parameter('timestamp_tolerance', 0.05)

        # ── Read parameters ──
        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').value
        training_topic = self.get_parameter('training_complete_topic').value
        self.prediction_steps = list(self.get_parameter('prediction_steps').value)
        self.dt = float(self.get_parameter('dt').value)
        self.min_velocity = float(self.get_parameter('min_velocity').value)
        self.log_interval = int(self.get_parameter('log_interval').value)
        self.csv_output_path = str(self.get_parameter('csv_output_path').value)
        self.buffer_size = int(self.get_parameter('data_buffer_size').value)
        self.timestamp_tolerance = float(self.get_parameter('timestamp_tolerance').value)
        self.enable_tire_force = bool(self.get_parameter('enable_tire_force_benchmark').value)
        tire_forces_topic = self.get_parameter('tire_forces_topic').value

        # ── Vehicle params dict (used by multi_step_predictor) ──
        self.vehicle_params = {
            'mass': float(self.get_parameter('mass').value),
            'I_z': float(self.get_parameter('I_z').value),
            'l_f': float(self.get_parameter('l_f').value),
            'l_r': float(self.get_parameter('l_r').value),
            'l_wb': float(self.get_parameter('l_wb').value),
        }

        # ── Pacejka parameters (will be overridden by training_complete) ──
        self.C_Pf = [float(v) for v in self.get_parameter('C_Pf').value]
        self.C_Pr = [float(v) for v in self.get_parameter('C_Pr').value]

        # ── State ──
        self.training_complete = False
        self.sample_count = 0

        # Timestamped circular buffers:
        #   state_buffer: deque of (timestamp_sec, v_x, v_y, omega, delta)
        self.state_buffer = deque(maxlen=self.buffer_size)

        # Latest ackermann for time sync pairing
        self.latest_ackermann_stamp = 0.0
        self.latest_delta = 0.0

        # ── Metrics: one OnlineBenchmark per (signal, step) ──
        self.bench = {}
        for step in self.prediction_steps:
            self.bench[f'vy_{step}step'] = OnlineBenchmark(f'v_y ({step}-step)')
            self.bench[f'omega_{step}step'] = OnlineBenchmark(f'omega ({step}-step)')

        # Tire force metrics (if enabled)
        self.tire_bench = {}
        if self.enable_tire_force:
            for label in ['fl_fy', 'fr_fy', 'rl_fy', 'rr_fy',
                          'front_sum_fy', 'rear_sum_fy', 'total_sum_fy']:
                self.tire_bench[label] = OnlineBenchmark(label)

        # ── Publishers ──
        self.metrics_pubs = {}
        for step in self.prediction_steps:
            self.metrics_pubs[f'vy_{step}step'] = self.create_publisher(
                Float64MultiArray,
                f'/estimation_benchmark/vy_{step}step_metrics', 10)
            self.metrics_pubs[f'omega_{step}step'] = self.create_publisher(
                Float64MultiArray,
                f'/estimation_benchmark/omega_{step}step_metrics', 10)

        self.summary_pub = self.create_publisher(
            String, '/estimation_benchmark/summary', 10)

        if self.enable_tire_force:
            self.tire_summary_pub = self.create_publisher(
                String, '/estimation_benchmark/tire_force_summary', 10)

        # ── Subscribers ──
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10)
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped, ackermann_topic, self._ackermann_cb, 10)

        # Training complete (transient local = latched)
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.training_sub = self.create_subscription(
            String, training_topic, self._training_complete_cb, qos_latched)

        # Tire forces (optional)
        self.tire_sub = None
        if self.enable_tire_force:
            try:
                from hellocm_msgs.msg import TireForcesArray
                self.tire_sub = self.create_subscription(
                    TireForcesArray, tire_forces_topic,
                    self._tire_forces_cb, 10)
                self.get_logger().info(
                    f'Tire force benchmark ENABLED, subscribing to: {tire_forces_topic}')
            except ImportError:
                self.get_logger().warn(
                    'hellocm_msgs not found — tire force benchmarking disabled.')
                self.enable_tire_force = False

        # ── CSV setup ──
        self.csv_file = None
        self.csv_writer = None
        self._setup_csv()

        # ── Log startup ──
        self.get_logger().info('EstimationBenchmarkNode started')
        self.get_logger().info(f'  Odom topic: {odom_topic}')
        self.get_logger().info(f'  Ackermann topic: {ackermann_topic}')
        self.get_logger().info(f'  Training signal: {training_topic}')
        self.get_logger().info(f'  Prediction steps: {self.prediction_steps}')
        self.get_logger().info(f'  dt: {self.dt} s')
        self.get_logger().info(f'  Tire force benchmark: {self.enable_tire_force}')
        self.get_logger().info('Waiting for training completion signal...')

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def _ackermann_cb(self, msg: AckermannDriveStamped):
        """Cache latest steering command with timestamp."""
        stamp = msg.header.stamp
        self.latest_ackermann_stamp = stamp.sec + stamp.nanosec * 1e-9
        self.latest_delta = msg.drive.steering_angle

    def _training_complete_cb(self, msg: String):
        """Activate benchmarking when On-Track-SysID training completes."""
        if self.training_complete:
            return  # Already active

        try:
            data = yaml.safe_load(msg.data)
            if isinstance(data, dict):
                if 'C_Pf' in data and len(data['C_Pf']) == 4:
                    self.C_Pf = [float(v) for v in data['C_Pf']]
                if 'C_Pr' in data and len(data['C_Pr']) == 4:
                    self.C_Pr = [float(v) for v in data['C_Pr']]
                self.get_logger().info(
                    f'Received identified Pacejka params — C_Pf: {self.C_Pf}, C_Pr: {self.C_Pr}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not parse training_complete message, using defaults: {e}')

        self.training_complete = True
        self.get_logger().info('Benchmarking ACTIVATED — model training complete.')

    def _odom_cb(self, msg: Odometry):
        """Process odometry: buffer state and run benchmarking if active."""
        stamp = msg.header.stamp
        t_sec = stamp.sec + stamp.nanosec * 1e-9

        v_x = msg.twist.twist.linear.x
        v_y = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z

        # Pair with latest ackermann by timestamp tolerance
        delta = self.latest_delta
        if abs(t_sec - self.latest_ackermann_stamp) > self.timestamp_tolerance:
            # No recent ackermann — still buffer but skip benchmarking
            self.state_buffer.append((t_sec, v_x, v_y, omega, delta))
            return

        # Append to buffer
        self.state_buffer.append((t_sec, v_x, v_y, omega, delta))

        if not self.training_complete:
            return

        if v_x < self.min_velocity:
            return

        # ── Run multi-step prediction benchmarking ──
        self._run_state_benchmarking()

    def _tire_forces_cb(self, msg):
        """Benchmark tire force estimation against CarMaker ground truth."""
        if not self.training_complete:
            return
        if not self.enable_tire_force:
            return

        fl = msg.front_left
        fr = msg.front_right
        rl = msg.rear_left
        rr = msg.rear_right

        tires = [fl, fr, rl, rr]
        # Skip if any tire is off-road or has negligible load
        for t in tires:
            if not t.on_road or abs(t.fz) < 50.0 or math.isnan(t.slip_angle):
                return

        vp = self.vehicle_params
        F_zf = vp['mass'] * 9.81 * vp['l_r'] / vp['l_wb']
        F_zr = vp['mass'] * 9.81 * vp['l_f'] / vp['l_wb']

        # Estimate using identified Pacejka
        fl_est = float(pacejka_formula(self.C_Pf, fl.slip_angle, fl.fz))
        fr_est = float(pacejka_formula(self.C_Pf, fr.slip_angle, fr.fz))
        rl_est = float(pacejka_formula(self.C_Pr, rl.slip_angle, rl.fz))
        rr_est = float(pacejka_formula(self.C_Pr, rr.slip_angle, rr.fz))

        # Update metrics
        self.tire_bench['fl_fy'].update(fl.fy, fl_est)
        self.tire_bench['fr_fy'].update(fr.fy, fr_est)
        self.tire_bench['rl_fy'].update(rl.fy, rl_est)
        self.tire_bench['rr_fy'].update(rr.fy, rr_est)

        front_gt = fl.fy + fr.fy
        rear_gt = rl.fy + rr.fy
        front_est = fl_est + fr_est
        rear_est = rl_est + rr_est

        self.tire_bench['front_sum_fy'].update(front_gt, front_est)
        self.tire_bench['rear_sum_fy'].update(rear_gt, rear_est)
        self.tire_bench['total_sum_fy'].update(front_gt + rear_gt, front_est + rear_est)

        # Periodic logging
        n = self.tire_bench['fl_fy'].metrics()['n_samples']
        if n > 0 and n % self.log_interval == 0:
            lines = [b.summary() for b in self.tire_bench.values()]
            summary_text = '\n'.join(lines)
            self.get_logger().info(f'\n=== Tire Force Benchmark ===\n{summary_text}')
            summary_msg = String()
            summary_msg.data = summary_text
            self.tire_summary_pub.publish(summary_msg)

    # ──────────────────────────────────────────────────────────────────
    # Core benchmarking logic
    # ──────────────────────────────────────────────────────────────────

    def _run_state_benchmarking(self):
        """Perform multi-step prediction and update metrics."""
        buf = self.state_buffer
        buf_len = len(buf)

        # Current measurement (last entry in buffer)
        _, v_x_now, v_y_now, omega_now, _ = buf[-1]

        for step in self.prediction_steps:
            # Need at least 'step' historical samples
            if buf_len <= step:
                continue

            # Look back 'step' entries
            idx = buf_len - 1 - step
            t_past, v_x_past, v_y_past, omega_past, delta_past = buf[idx]

            # Predict from historical state forward 'step' steps
            v_y_pred, omega_pred = predict_multi_step(
                v_x_past, v_y_past, omega_past, delta_past,
                self.C_Pf, self.C_Pr,
                self.vehicle_params, self.dt, step,
            )

            # Update metrics
            self.bench[f'vy_{step}step'].update(v_y_now, v_y_pred)
            self.bench[f'omega_{step}step'].update(omega_now, omega_pred)

            # Publish per-signal metrics
            for signal in ['vy', 'omega']:
                key = f'{signal}_{step}step'
                msg = Float64MultiArray()
                msg.data = self.bench[key].metrics_array()
                self.metrics_pubs[key].publish(msg)

        # ── CSV logging ──
        if self.csv_writer is not None:
            row = [buf[-1][0]]  # timestamp
            for step in self.prediction_steps:
                if buf_len <= step:
                    row.extend([float('nan')] * 7 * 2)
                    continue
                for signal in ['vy', 'omega']:
                    row.extend(self.bench[f'{signal}_{step}step'].metrics_array())
            self.csv_writer.writerow(row)

        # ── Periodic summary ──
        self.sample_count += 1
        if self.sample_count % self.log_interval == 0:
            lines = []
            for step in self.prediction_steps:
                lines.append(self.bench[f'vy_{step}step'].summary())
                lines.append(self.bench[f'omega_{step}step'].summary())
            summary_text = '\n'.join(lines)
            self.get_logger().info(f'\n=== Estimation Benchmark ===\n{summary_text}')

            summary_msg = String()
            summary_msg.data = summary_text
            self.summary_pub.publish(summary_msg)

    # ──────────────────────────────────────────────────────────────────
    # CSV setup
    # ──────────────────────────────────────────────────────────────────

    def _setup_csv(self):
        """Create CSV file with header if csv_output_path is set."""
        if self.csv_output_path == '':
            return
        output_path = Path(self.csv_output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = output_path.open('w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)

        header = ['timestamp']
        for step in self.prediction_steps:
            for signal in ['vy', 'omega']:
                header.extend([
                    f'{signal}_{step}step_rmse',
                    f'{signal}_{step}step_mae',
                    f'{signal}_{step}step_nrmse',
                    f'{signal}_{step}step_maxae',
                    f'{signal}_{step}step_bias',
                    f'{signal}_{step}step_std',
                    f'{signal}_{step}step_r2',
                ])
        self.csv_writer.writerow(header)
        self.get_logger().info(f'CSV logging enabled: {self.csv_output_path}')

    def destroy_node(self):
        """Flush and close CSV on shutdown."""
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstimationBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
