"""Pure-logic unit tests for online_metrics.py - no rclpy needed (mirrors
tire_force_benchmark/test/test_queue_alignment.py's convention of testing the
non-ROS logic module directly, but this module has zero ROS dependency at all).
"""
import math

from adaptive_controller_benchmark.online_metrics import LapTracker
from adaptive_controller_benchmark.online_metrics import SignalStats
from adaptive_controller_benchmark.online_metrics import SwitchTracker


def test_signal_stats_empty():
    stats = SignalStats()
    m = stats.metrics()
    assert m == {'n': 0, 'rms': 0.0, 'max_abs': 0.0, 'itae': 0.0}


def test_signal_stats_rms_max_itae():
    stats = SignalStats()
    stats.update(value=3.0, dt=1.0, t_since_reset=0.0)
    stats.update(value=4.0, dt=1.0, t_since_reset=1.0)
    stats.update(value=-4.0, dt=1.0, t_since_reset=2.0)

    m = stats.metrics()
    assert m['n'] == 3
    assert math.isclose(m['rms'], math.sqrt((9.0 + 16.0 + 16.0) / 3.0))
    assert m['max_abs'] == 4.0
    # itae = sum(dt * t * |value|) = 1*0*3 + 1*1*4 + 1*2*4 = 0 + 4 + 8 = 12
    assert math.isclose(m['itae'], 12.0)


def test_signal_stats_reset():
    stats = SignalStats()
    stats.update(1.0, 1.0, 0.0)
    stats.reset()
    assert stats.metrics()['n'] == 0


def test_switch_tracker_dwell_and_transitions():
    tracker = SwitchTracker()
    ticks = [
        (0.0, 'BOOTSTRAP_PP', 0.0, 0.0),
        (1.0, 'RUNNING_PP', 1.0, 0.1),
        (2.0, 'RUNNING_PP', 1.0, 0.1),
        (3.0, 'SWITCHING_TO_MPC', 2.0, 0.2),
        (4.0, 'RUNNING_MPC', 3.0, 0.2),
    ]
    for t, state, v_cmd, steering in ticks:
        tracker.update(t, state, v_cmd, steering)
    tracker.finalize(t_end=5.0)

    assert tracker.dwell_times['BOOTSTRAP_PP'] == [1.0]
    assert tracker.dwell_times['RUNNING_PP'] == [2.0]
    assert tracker.dwell_times['SWITCHING_TO_MPC'] == [1.0]
    assert tracker.dwell_times['RUNNING_MPC'] == [1.0]

    assert tracker.transition_counts[('BOOTSTRAP_PP', 'RUNNING_PP')] == 1
    assert tracker.transition_counts[('RUNNING_PP', 'SWITCHING_TO_MPC')] == 1
    assert tracker.transition_counts[('SWITCHING_TO_MPC', 'RUNNING_MPC')] == 1

    assert len(tracker.episodes) == 1
    ep = tracker.episodes[0]
    assert ep['to_state'] == 'SWITCHING_TO_MPC'
    assert math.isclose(ep['exit_steering'], 0.1)
    assert math.isclose(ep['entry_steering'], 0.2)
    assert ep['samples'] == [(0.0, 2.0, 0.2, None)]

    summary = tracker.summary()
    assert summary['n_episodes'] == 1
    assert summary['dwell']['RUNNING_PP']['mean'] == 2.0


def test_switch_tracker_no_transitions_still_finalizes():
    tracker = SwitchTracker()
    tracker.update(0.0, 'RUNNING_PP', 1.0, 0.0)
    tracker.update(1.0, 'RUNNING_PP', 1.0, 0.0)
    tracker.finalize(t_end=3.0)
    assert tracker.dwell_times['RUNNING_PP'] == [3.0]
    assert tracker.episodes == []


def test_lap_tracker_detects_wraparound():
    tracker = LapTracker()
    tracker.set_waypoints(x_m=[0.0, 1.0, 2.0, 3.0], y_m=[0.0, 0.0, 0.0, 0.0], s_m=[0.0, 1.0, 2.0, 3.0])
    assert tracker.ready()

    tracker.update(t=0.0, x=0.0, y=0.0)
    tracker.update(t=1.0, x=1.0, y=0.0)
    tracker.update(t=2.0, x=2.0, y=0.0)
    tracker.update(t=3.0, x=3.0, y=0.0)
    assert tracker.lap_times == []

    tracker.update(t=4.0, x=0.0, y=0.0)
    assert tracker.lap_times == [4.0]


def test_lap_tracker_not_ready_without_waypoints():
    tracker = LapTracker()
    assert not tracker.ready()
    tracker.update(t=0.0, x=0.0, y=0.0)  # no-op, must not raise
    assert tracker.lap_times == []
