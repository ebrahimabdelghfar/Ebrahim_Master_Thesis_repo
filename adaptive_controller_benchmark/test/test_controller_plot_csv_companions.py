"""Every exported figure must ship a same-named CSV holding its numbers.

benchmark_runner/compare_scenarios.py reads those CSVs rather than re-deriving
any metric, so a figure without one silently drops out of the cross-scenario
comparison instead of failing loudly.

Node.create_subscription is monkeypatched so the node can be built and driven
directly, without a live ROS graph - the same approach
tire_force_benchmark/test/test_queue_alignment.py already uses.
"""
import csv
import math
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from adaptive_controller_benchmark.adaptive_controller_benchmark_node import (  # noqa: E402
    AdaptiveControllerBenchmarkNode)

TRACK_RADIUS = 20.0
SAMPLES_PER_LAP = 60


class _FakeSubscription:
    pass


class NodeUnderTest:

    def __init__(self, overrides):
        self.overrides = overrides
        self._orig = Node.create_subscription
        self.node = None

    def __enter__(self):
        args = ['--ros-args']
        for key, value in self.overrides.items():
            args += ['-p', f'{key}:={value}']
        rclpy.init(args=args)
        Node.create_subscription = lambda self, msg_type, topic, cb, qos: _FakeSubscription()
        try:
            self.node = AdaptiveControllerBenchmarkNode()
        except Exception:
            Node.create_subscription = self._orig
            rclpy.shutdown()
            raise
        return self.node

    def __exit__(self, exc_type, exc_value, traceback):
        Node.create_subscription = self._orig
        if self.node is not None:
            self.node.destroy_node()
        rclpy.shutdown()
        return False


def _feed_waypoints(node):
    """A closed circular raceline, so s_m wraps and laps can be detected."""
    class _Waypoint:
        def __init__(self, x, y, s):
            self.x_m, self.y_m, self.s_m = x, y, s

    class _Array:
        pass

    msg = _Array()
    msg.waypoints = []
    total = 2.0 * math.pi * TRACK_RADIUS
    for i in range(SAMPLES_PER_LAP):
        angle = 2.0 * math.pi * i / SAMPLES_PER_LAP
        msg.waypoints.append(_Waypoint(
            TRACK_RADIUS * math.cos(angle), TRACK_RADIUS * math.sin(angle),
            total * i / SAMPLES_PER_LAP))
    node._waypoint_cb(msg)


def _drive(node, laps=2, start_fraction=0.6):
    """Drive `laps` full circuits, entering the track partway round.

    `start_fraction` reproduces the real spawn, which sits behind the
    raceline's s = 0.
    """
    states = ['RUNNING_PP'] * 20 + ['SWITCHING_TO_MPC'] * 3 + ['RUNNING_MPC'] * 1000
    steps = int(SAMPLES_PER_LAP * (laps + 1 - start_fraction)) + 1
    for step in range(steps):
        fraction = start_fraction + step / SAMPLES_PER_LAP
        angle = 2.0 * math.pi * fraction

        odom = Odometry()
        odom.pose.pose.position.x = TRACK_RADIUS * math.cos(angle)
        odom.pose.pose.position.y = TRACK_RADIUS * math.sin(angle)
        odom.twist.twist.linear.x = 9.0
        node._odom_cb(odom)

        node._lateral_error_cb(Float64(data=0.1 * math.sin(angle)))
        node._heading_error_cb(Float64(data=0.01 * math.cos(angle)))
        node._solve_time_cb(Float64(data=1.5))
        node._state_cb(String(data=states[min(step, len(states) - 1)]))


def _read(path):
    with Path(path).open(newline='', encoding='utf-8') as handle:
        return list(csv.reader(handle))


def test_every_exported_png_has_a_csv_companion(tmp_path):
    with NodeUnderTest({'plot_output_dir': str(tmp_path)}) as node:
        _feed_waypoints(node)
        _drive(node, laps=2)
        node._export_plots()
        node.plot_output_dir = ''  # destroy_node() would otherwise export again

    pngs = sorted(p.name for p in tmp_path.glob('*.png'))
    assert pngs, 'export produced no figures at all'
    assert 'lap_times.png' in pngs
    assert 'track_lap_01.png' in pngs
    for png in tmp_path.glob('*.png'):
        companion = png.with_suffix('.csv')
        assert companion.is_file(), f'{png.name} has no CSV companion'
        assert len(_read(companion)) >= 2, f'{companion.name} has a header but no data'


def test_metrics_summary_csv_is_tidy_and_carries_the_headline_numbers(tmp_path):
    with NodeUnderTest({'plot_output_dir': str(tmp_path)}) as node:
        _feed_waypoints(node)
        _drive(node, laps=2)
        node._export_plots()
        node.plot_output_dir = ''

    rows = _read(tmp_path / 'metrics_summary_table.csv')
    assert rows[0] == ['section', 'key', 'metric', 'value']
    lookup = {(r[0], r[1], r[2]): r[3] for r in rows[1:]}
    assert ('tracking', 'Overall', 'rms_e_y_m') in lookup
    assert ('tracking', 'Overall', 'itae_e_y') in lookup
    assert ('summary', '', 'laps') in lookup
    # The out-lap is not a lap: 2 circuits after entering partway round means
    # one timed lap plus an incomplete second one.
    assert int(lookup[('summary', '', 'laps')]) >= 1


def test_lap_times_csv_row_count_matches_completed_laps(tmp_path):
    with NodeUnderTest({'plot_output_dir': str(tmp_path)}) as node:
        _feed_waypoints(node)
        _drive(node, laps=3)
        completed = len(node.lap_tracker.lap_times)
        node._export_plots()
        node.plot_output_dir = ''

    rows = _read(tmp_path / 'lap_times.csv')
    assert len(rows) - 1 == completed
