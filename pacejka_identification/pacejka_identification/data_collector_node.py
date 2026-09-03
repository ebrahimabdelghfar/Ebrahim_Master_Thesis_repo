#!/usr/bin/env python3
"""
ROS2 node that subscribes to /sim/feedback/tire_forces
(sim_manager_msgs/TireForces — CARLA's own per-wheel telemetry) and collects
raw tire data for subsequent Pacejka coefficient identification.
"""

import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from sim_manager_msgs.msg import TireForces


# Wheel labels
WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

# Columns stored per wheel
PER_WHEEL_COLS = [
    'slip_angle', 'slip_ratio', 'fz', 'fy', 'fx', 'wheel_torque',
    'tire_friction', 'wheel_speed',
]


class DataCollectorNode(Node):
    """Collect per-wheel tire telemetry from the simulator and export to CSV."""

    def __init__(self):
        super().__init__('data_collector_node')

        # ---------- Parameters ----------
        self.declare_parameter('tire_forces_topic', '/sim/feedback/tire_forces')
        self.declare_parameter('duration_seconds', 60)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('min_speed', 2.0)
        self.declare_parameter('min_fz_threshold', 50.0)
        self.declare_parameter('csv_export', True)
        self.declare_parameter('csv_path', '')

        self.topic = self.get_parameter('tire_forces_topic').value
        self.duration = int(self.get_parameter('duration_seconds').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.speed_gate_enabled = bool(self.odom_topic) and self.min_speed > 0.0
        self.min_fz = float(self.get_parameter('min_fz_threshold').value)
        self.csv_export = bool(self.get_parameter('csv_export').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        # ---------- Storage ----------
        # Dict of lists per wheel: {'FL': {'slip_angle': [], ...}, ...}
        self.data = {w: {c: [] for c in PER_WHEEL_COLS} for w in WHEEL_NAMES}
        self.sample_count = 0
        self.kept_count = 0
        self.collection_complete = False
        self.t_start = None
        self._last_progress_t = 0.0
        self.speed = None

        # ---------- Subscriptions ----------
        self.sub = self.create_subscription(
            TireForces,
            self.topic,
            self._tire_cb,
            10,
        )
        if self.speed_gate_enabled:
            # /odom is BEST_EFFORT on the CARLA bridge — a RELIABLE
            # subscription is QoS-incompatible and gets nothing.
            self.create_subscription(
                Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data
            )

        self.get_logger().info(f'DataCollectorNode started — subscribing to {self.topic}')
        self.get_logger().info(f'Collecting for {self.duration}s of wall clock')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.speed = math.hypot(v.x, v.y)

    def _tire_cb(self, msg: TireForces):
        if self.collection_complete:
            return

        self.sample_count += 1
        if self.t_start is None:
            self.t_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9

        moving = not self.speed_gate_enabled or (
            self.speed is not None and self.speed >= self.min_speed
        )
        if moving:
            names = list(msg.wheel_names) if len(msg.wheel_names) == 4 else WHEEL_NAMES
            for i, wheel_name in enumerate(names):
                if wheel_name not in self.data:
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
                if not self._valid_sample(sample):
                    continue
                for col, val in sample.items():
                    self.data[wheel_name][col].append(val)
            self.kept_count += 1

        # Progress logging (~every 5 % of the collection window)
        if elapsed - self._last_progress_t >= max(1.0, self.duration / 20.0):
            self._last_progress_t = elapsed
            pct = min(100.0, elapsed / self.duration * 100)
            bar_len = 25
            filled = int(bar_len * pct / 100)
            bar = '=' * filled + '-' * (bar_len - filled)
            self.get_logger().info(
                f'Collecting: [{bar}] {pct:.1f}%  '
                f'({elapsed:.0f}/{self.duration}s, {self.kept_count}/{self.sample_count} msgs kept)'
            )

        if elapsed >= self.duration:
            self.collection_complete = True
            self.get_logger().info(
                f'Data collection complete — {self.kept_count}/{self.sample_count} messages kept'
            )
            if self.csv_export:
                self.export_csv()

    def _valid_sample(self, sample: dict) -> bool:
        if abs(sample['fz']) < self.min_fz:
            return False
        # Below 0.5 m/s the publisher zeroes slip_angle, slip_ratio and both
        # forces while Fz stays live, so min_fz alone lets those through.
        if sample['slip_angle'] == 0.0 and sample['slip_ratio'] == 0.0 and sample['fy'] == 0.0:
            return False
        if any(math.isnan(v) or math.isinf(v) for v in sample.values()):
            return False
        return True

    # ------------------------------------------------------------------
    # Data access helpers (used by identification_node)
    # ------------------------------------------------------------------

    def get_numpy(self, wheel: str, column: str) -> np.ndarray:
        """Return collected data for a specific wheel and column as ndarray."""
        return np.array(self.data[wheel][column], dtype=np.float64)

    def get_axle_numpy(self, axle: str, column: str) -> np.ndarray:
        """Return pooled data for an axle ('front' or 'rear')."""
        if axle == 'front':
            wheels = ['FL', 'FR']
        else:
            wheels = ['RL', 'RR']
        return np.concatenate([self.get_numpy(w, column) for w in wheels])

    def get_all_numpy(self, column: str) -> np.ndarray:
        """Return pooled data from all four wheels."""
        return np.concatenate([self.get_numpy(w, column) for w in WHEEL_NAMES])

    @property
    def is_complete(self) -> bool:
        return self.collection_complete

    # ------------------------------------------------------------------
    # CSV export
    # ------------------------------------------------------------------

    def export_csv(self):
        path = self.csv_path
        if not path:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = os.path.expanduser(f'~/pacejka_id_data_{ts}.csv')

        os.makedirs(os.path.dirname(path) if os.path.dirname(path) else '.', exist_ok=True)

        # Build column headers: FL_slip_angle, FL_slip_ratio, ...
        headers = []
        for w in WHEEL_NAMES:
            for c in PER_WHEEL_COLS:
                headers.append(f'{w}_{c}')

        # Determine the minimum length across all wheels
        min_len = min(len(self.data[w]['slip_angle']) for w in WHEEL_NAMES)

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)
            for i in range(min_len):
                row = []
                for w in WHEEL_NAMES:
                    for c in PER_WHEEL_COLS:
                        row.append(self.data[w][c][i])
                writer.writerow(row)

        self.get_logger().info(f'Dataset exported to: {path}')
        return path


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
