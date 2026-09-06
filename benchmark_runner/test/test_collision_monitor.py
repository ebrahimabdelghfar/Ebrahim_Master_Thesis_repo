"""Which collisions end a run, and which are just paint.

`_on_event` is pure arithmetic over a message plus a CSV write, so it is tested
without a ROS graph: the node is built with its subscription stubbed.
"""
import sys
from pathlib import Path

import pytest
import rclpy
from carla_msgs.msg import CarlaCollisionEvent
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from collision_monitor import CollisionMonitor  # noqa: E402

THRESHOLD = 200.0


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def monitor(tmp_path, monkeypatch):
    # No bridge to subscribe to; the callback is driven directly below.
    monkeypatch.setattr(Node, 'create_subscription', lambda *a, **k: None)
    node = CollisionMonitor(THRESHOLD, tmp_path / 'collisions.csv')
    yield node
    node.close()
    node.destroy_node()


# The spawn drop, measured: the car is spawned above the road and settles onto
# it, firing 18 events in ~0.7 s. This is the largest of them. Almost all of it
# is vertical, which is what separates it from a crash.
SPAWN_DROP = (-0.5826, -3.6193, 267.9557)


def _event(magnitude, actor_id=42, t=1.5):
    msg = CarlaCollisionEvent()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int((t - int(t)) * 1e9)
    msg.other_actor_id = actor_id
    # Along one horizontal axis, so the magnitude is exactly what was asked for.
    msg.normal_impulse.x = float(magnitude)
    return msg


def _xyz(x, y, z, actor_id=0):
    msg = _event(0.0, actor_id=actor_id)
    msg.normal_impulse.x, msg.normal_impulse.y, msg.normal_impulse.z = x, y, z
    return msg


def test_a_graze_does_not_fail_the_run(monitor):
    monitor._on_event(_event(THRESHOLD - 1.0))
    assert monitor.crash() is None
    assert monitor.count() == 1


def test_a_hit_fails_the_run(monitor):
    monitor._on_event(_event(THRESHOLD + 1.0, actor_id=7))
    crash = monitor.crash()
    assert crash is not None
    assert 'actor 7' in crash and '201.0 N*s' in crash


def test_exactly_at_the_threshold_fails(monitor):
    monitor._on_event(_event(THRESHOLD))
    assert monitor.crash() is not None


def test_the_first_hit_is_the_one_reported(monitor):
    # CARLA fires one event per contact frame; the burst is all one crash.
    monitor._on_event(_event(300.0, actor_id=1))
    monitor._on_event(_event(900.0, actor_id=2))
    assert 'actor 1' in monitor.crash()
    assert monitor.count() == 3 - 1  # both events counted


def test_magnitude_combines_both_horizontal_axes(monitor):
    # 3-4-5: horizontal magnitude exactly 200, on the line.
    monitor._on_event(_xyz(120.0, -160.0, 0.0))
    assert monitor.crash() is not None


def test_the_spawn_drop_does_not_fail_the_run(monitor):
    """The regression this whole horizontal-only rule exists for."""
    monitor._on_event(_xyz(*SPAWN_DROP))
    assert monitor.crash() is None


def test_a_vertical_impulse_alone_never_fails(monitor):
    # Landing on the road carries the whole vehicle weight through z.
    monitor._on_event(_xyz(0.0, 0.0, 10 * THRESHOLD))
    assert monitor.crash() is None


def test_a_wall_hit_fails_despite_reporting_static_geometry(monitor):
    # Barriers report other_actor_id 0 exactly like the road does, so the id
    # cannot be the discriminator - only the direction can.
    monitor._on_event(_xyz(0.0, -250.0, 30.0, actor_id=0))
    assert monitor.crash() is not None


def test_every_contact_reaches_the_csv(monitor, tmp_path):
    monitor._on_event(_event(10.0))
    monitor._on_event(_event(500.0))
    monitor._csv_file.flush()  # not close(): the fixture still owns that
    rows = (tmp_path / 'collisions.csv').read_text().strip().splitlines()
    assert len(rows) == 3  # header + 2
    assert rows[1].endswith(',0') and rows[2].endswith(',1')  # over_threshold flag
    assert 'impulse_horizontal' in rows[0]
