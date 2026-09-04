"""The three LLA-MPC friction schedules, and where their steps land.

`_mu_at` is pure arithmetic over a clock and the lap monitor, so it is tested
without a ROS graph: the node is built with its publisher and timer stubbed.
"""
import sys
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from friction_schedule import DECAY_PER_S  # noqa: E402
from friction_schedule import MU_FLOOR  # noqa: E402
from friction_schedule import STEP_FACTOR  # noqa: E402
from friction_schedule import FrictionSchedule  # noqa: E402

NOMINAL = 1.5


class _FakeLapMonitor:
    def __init__(self, completed=0, wraparounds=0, fraction=None):
        self._completed = completed
        self._wraparounds = wraparounds
        self._fraction = fraction

    def completed_laps(self):
        return self._completed

    def wraparounds(self):
        return self._wraparounds

    def lap_fraction(self):
        return self._fraction


class _FakeTimer:
    def cancel(self):
        pass


def _build(schedule, tmp_path, lap_monitor=None):
    original_pub = Node.create_publisher
    original_timer = Node.create_timer
    Node.create_publisher = lambda self, msg_type, topic, qos: _FakePublisher()
    Node.create_timer = lambda self, period, cb: _FakeTimer()
    try:
        return FrictionSchedule(
            schedule, NOMINAL, tmp_path / 'mu_commanded.csv', lap_monitor=lap_monitor)
    finally:
        Node.create_publisher = original_pub
        Node.create_timer = original_timer


class _FakePublisher:
    def publish(self, msg):
        pass


@pytest.fixture(autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_constant_holds_the_configured_value(tmp_path):
    node = _build('constant', tmp_path)
    assert node._mu_at(0.0) == NOMINAL
    assert node._mu_at(120.0) == NOMINAL
    node.close()


def test_decay_is_two_percent_per_second_and_never_reaches_zero(tmp_path):
    node = _build('decay_2pct_s', tmp_path)
    assert node._mu_at(0.0) == pytest.approx(NOMINAL)
    assert node._mu_at(10.0) == pytest.approx(NOMINAL * (1.0 - 10.0 * DECAY_PER_S))
    assert node._mu_at(10_000.0) == pytest.approx(MU_FLOOR)
    node.close()


def test_step_at_end_of_lap_1_ignores_the_out_lap(tmp_path):
    # One wraparound has happened, but no TIMED lap is complete yet - that is
    # the out-lap ending, and it must not trigger the step.
    monitor = _FakeLapMonitor(completed=0, wraparounds=1)
    node = _build('step40_end_lap1', tmp_path, monitor)
    assert node._mu_at(5.0) == pytest.approx(NOMINAL)

    monitor._completed = 1
    monitor._wraparounds = 2
    assert node._mu_at(6.0) == pytest.approx(STEP_FACTOR * NOMINAL)
    node.close()


def test_step_is_latched_once_taken(tmp_path):
    monitor = _FakeLapMonitor(completed=1, wraparounds=2)
    node = _build('step40_end_lap1', tmp_path, monitor)
    assert node._mu_at(6.0) == pytest.approx(STEP_FACTOR * NOMINAL)

    # A monitor that somehow reports fewer laps must not restore grip.
    monitor._completed = 0
    assert node._mu_at(7.0) == pytest.approx(STEP_FACTOR * NOMINAL)
    node.close()


def test_step_mid_lap_1_fires_at_half_arc_length(tmp_path):
    monitor = _FakeLapMonitor(completed=0, wraparounds=1, fraction=0.30)
    node = _build('step40_mid_lap1', tmp_path, monitor)
    assert node._mu_at(3.0) == pytest.approx(NOMINAL)

    monitor._fraction = 0.55
    assert node._mu_at(4.0) == pytest.approx(STEP_FACTOR * NOMINAL)
    node.close()


def test_step_mid_lap_1_does_not_fire_during_the_out_lap(tmp_path):
    # Halfway round, but still before the first s=0 crossing.
    monitor = _FakeLapMonitor(completed=0, wraparounds=0, fraction=0.80)
    node = _build('step40_mid_lap1', tmp_path, monitor)
    assert node._mu_at(2.0) == pytest.approx(NOMINAL)
    node.close()


def test_unknown_schedule_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        _build('wet_track', tmp_path)
