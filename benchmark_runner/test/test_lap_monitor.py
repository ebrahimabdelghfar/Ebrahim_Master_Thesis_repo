"""The out-lap must not count.

The car spawns behind the raceline's s = 0, so the drive to the start line is
not a lap. `wait_for_laps(n)` therefore waits on n TIMED laps, i.e. n + 1
`s_m` wraparounds - stopping on `current_lap >= n` would end the run a full lap
early and leave the last lap incomplete in every figure.
"""
import math
import sys
import threading
from pathlib import Path

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lap_monitor import LapMonitor  # noqa: E402

TRACK_RADIUS = 20.0
SAMPLES_PER_LAP = 120


class _FakeSubscription:
    pass


@pytest.fixture
def monitor():
    original = Node.create_subscription
    rclpy.init()
    Node.create_subscription = lambda self, msg_type, topic, cb, qos: _FakeSubscription()
    try:
        node = LapMonitor()
        _feed_waypoints(node)
        yield node
    finally:
        Node.create_subscription = original
        node.destroy_node()
        rclpy.shutdown()


def _feed_waypoints(node):
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


def _step(node, index, fraction):
    """One odometry sample at `fraction` of the way round the circuit."""
    angle = 2.0 * math.pi * fraction
    msg = Odometry()
    msg.header.stamp.sec = index // 50
    msg.header.stamp.nanosec = (index % 50) * 20_000_000
    msg.pose.pose.position.x = TRACK_RADIUS * math.cos(angle)
    msg.pose.pose.position.y = TRACK_RADIUS * math.sin(angle)
    node._odom_cb(msg)


def _drive(node, circuits, start_fraction=0.6):
    """Enter the track partway round, mirroring the real spawn point."""
    steps = int(SAMPLES_PER_LAP * circuits)
    for step in range(steps + 1):
        _step(node, step, start_fraction + step / SAMPLES_PER_LAP)


def test_out_lap_is_not_counted(monitor):
    # Just short of the first s=0 crossing: still the out-lap.
    _drive(monitor, circuits=0.3)
    assert monitor.wraparounds() == 0
    assert monitor.completed_laps() == 0


def test_first_crossing_starts_the_clock_but_completes_no_lap(monitor):
    _drive(monitor, circuits=0.8)
    assert monitor.wraparounds() == 1
    assert monitor.completed_laps() == 0, 'the out-lap was counted as lap 1'


def test_two_timed_laps_need_three_wraparounds(monitor):
    _drive(monitor, circuits=2.8)
    assert monitor.wraparounds() == 3
    assert monitor.completed_laps() == 2


def test_wait_for_laps_releases_on_the_nth_timed_lap(monitor):
    released = threading.Event()

    def driver():
        for step in range(int(SAMPLES_PER_LAP * 2.8) + 1):
            if released.is_set():
                return
            _step(monitor, step, 0.6 + step / SAMPLES_PER_LAP)

    thread = threading.Thread(target=driver)
    thread.start()
    try:
        assert monitor.wait_for_laps(2, timeout_s=20.0, poll_s=0.01)
    finally:
        released.set()
        thread.join()
    assert monitor.completed_laps() >= 2


def test_lap_fraction_tracks_arc_length(monitor):
    assert monitor.lap_fraction() is None
    _drive(monitor, circuits=0.2, start_fraction=0.0)
    assert monitor.lap_fraction() == pytest.approx(0.2, abs=0.02)
