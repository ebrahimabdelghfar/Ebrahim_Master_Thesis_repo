#!/usr/bin/env python3
"""
ROS2 node that subscribes to /tire_forces and collects raw tire data
for subsequent Pacejka coefficient identification.
"""

import csv
import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node

from hellocm_msgs.msg import TireForcesArray


# Wheel labels
WHEEL_NAMES = ['FL', 'FR', 'RL', 'RR']

# Columns stored per wheel
PER_WHEEL_COLS = [
    'slip_angle', 'long_slip', 'fz', 'fy', 'fx', 'mz',
    'inclination_angle', 'mu_road',
]


class DataCollectorNode(Node):
    """Collect tire-force data from CarMaker and export to CSV."""

    def __init__(self):
        super().__init__('data_collector_node')

        # ---------- Parameters ----------
        self.declare_parameter('tire_forces_topic', '/tire_forces')
        self.declare_parameter('duration_seconds', 60)
        self.declare_parameter('min_velocity', 0.5)
        self.declare_parameter('require_on_road', True)
        self.declare_parameter('min_fz_threshold', 50.0)
        self.declare_parameter('csv_export', True)
        self.declare_parameter('csv_path', '')

        self.topic = self.get_parameter('tire_forces_topic').value
        self.duration = int(self.get_parameter('duration_seconds').value)
        self.min_vel = float(self.get_parameter('min_velocity').value)
        self.require_on_road = bool(self.get_parameter('require_on_road').value)
        self.min_fz = float(self.get_parameter('min_fz_threshold').value)
        self.csv_export = bool(self.get_parameter('csv_export').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        # ---------- Storage ----------
        # Dict of lists per wheel: {'FL': {'slip_angle': [], ...}, ...}
        self.data = {w: {c: [] for c in PER_WHEEL_COLS} for w in WHEEL_NAMES}
        self.sample_count = 0
        self.collection_complete = False

        # We estimate the expected number of samples at 100 Hz
        self.expected_samples = self.duration * 100

        # ---------- Subscription ----------
        self.sub = self.create_subscription(
            TireForcesArray,
            self.topic,
            self._tire_cb,
            10,
        )

        self.get_logger().info(f'DataCollectorNode started — subscribing to {self.topic}')
        self.get_logger().info(f'Collecting for ~{self.duration}s ({self.expected_samples} expected samples)')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _tire_cb(self, msg: TireForcesArray):
        if self.collection_complete:
            return

        tires = [msg.front_left, msg.front_right, msg.rear_left, msg.rear_right]

        for wheel_name, tire in zip(WHEEL_NAMES, tires):
            if not self._valid_sample(tire):
                continue

            d = self.data[wheel_name]
            d['slip_angle'].append(tire.slip_angle)
            d['long_slip'].append(tire.long_slip)
            d['fz'].append(tire.fz)
            d['fy'].append(tire.fy)
            d['fx'].append(tire.fx)
            d['mz'].append(tire.mz)
            d['inclination_angle'].append(tire.inclination_angle)
            d['mu_road'].append(tire.mu_road)

        self.sample_count += 1

        # Progress logging (~every 5 %)
        interval = max(1, self.expected_samples // 20)
        if self.sample_count % interval == 0 or self.sample_count >= self.expected_samples:
            pct = min(100.0, self.sample_count / self.expected_samples * 100)
            bar_len = 25
            filled = int(bar_len * pct / 100)
            bar = '=' * filled + '-' * (bar_len - filled)
            self.get_logger().info(
                f'Collecting: [{bar}] {pct:.1f}%  ({self.sample_count}/{self.expected_samples})'
            )

        if self.sample_count >= self.expected_samples:
            self.collection_complete = True
            self.get_logger().info('Data collection complete!')
            if self.csv_export:
                self.export_csv()

    def _valid_sample(self, tire) -> bool:
        if self.require_on_road and not tire.on_road:
            return False
        if abs(tire.fz) < self.min_fz:
            return False
        if tire.belt_velocity < self.min_vel:
            return False
        if any(math.isnan(v) for v in [tire.slip_angle, tire.long_slip,
                                        tire.fz, tire.fy, tire.fx, tire.mz]):
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

        # Build column headers: FL_slip_angle, FL_long_slip, ...
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
