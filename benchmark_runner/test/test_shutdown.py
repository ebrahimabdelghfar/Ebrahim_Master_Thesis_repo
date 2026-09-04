"""Ctrl-C must stop the launches, in order, and leave nothing running.

The failure this pins: rclpy's default SIGINT handler shuts the ROS context down,
the teardown then throws while publishing the nominal friction back, and the
`finally` never reaches the launches - so the identification and control stacks
keep driving the car after the runner has exited.
"""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import run_benchmark  # noqa: E402
from run_benchmark import Aborted  # noqa: E402
from run_benchmark import Launch  # noqa: E402
from run_benchmark import SHUTDOWN_ORDER  # noqa: E402


@pytest.fixture(autouse=True)
def clean_module_state():
    run_benchmark._INTERRUPTED.clear()
    run_benchmark._LIVE_LAUNCHES.clear()
    original = signal.getsignal(signal.SIGINT)
    yield
    signal.signal(signal.SIGINT, original)
    for launch in list(run_benchmark._LIVE_LAUNCHES):
        launch.shutdown(timeout_s=5.0)
    run_benchmark._INTERRUPTED.clear()
    run_benchmark._LIVE_LAUNCHES.clear()


def _sleeper(name, tmp_path, seconds=600):
    launch = Launch(name, 'unused', tmp_path / f'{name}.log')
    # Replace the ros2 launch invocation; `exec` so the Popen pid is the sleep
    # itself, exactly as it is the ros2 launch process in real use.
    launch.command = f'exec sleep {seconds}'
    launch.start()
    return launch


# ---------------- the signal handler ----------------

def test_first_interrupt_sets_the_flag_without_raising():
    run_benchmark._install_signal_handlers()
    os.kill(os.getpid(), signal.SIGINT)  # must not raise KeyboardInterrupt
    assert run_benchmark._INTERRUPTED.is_set()


def test_second_interrupt_raises_so_the_operator_can_escape():
    run_benchmark._install_signal_handlers()
    os.kill(os.getpid(), signal.SIGINT)
    with pytest.raises(KeyboardInterrupt):
        os.kill(os.getpid(), signal.SIGINT)


def test_interrupt_aborts_the_lap_wait():
    runner = object.__new__(run_benchmark.BenchmarkRunner)
    run_benchmark._INTERRUPTED.set()
    with pytest.raises(Aborted):
        runner._require_alive({})


# ---------------- the teardown ----------------

class _FakeBridge:
    def ensure_unconfigured(self):
        pass

    def state_name(self):
        return 'unconfigured'


class _ExplodingFriction:
    """Stands in for a friction node whose context has gone away."""

    def restore_nominal(self):
        raise RuntimeError('publisher context is invalid')


def _runner_with(bridge=None):
    runner = object.__new__(run_benchmark.BenchmarkRunner)
    runner.bridge = bridge or _FakeBridge()
    # _verify_clean pgreps the whole machine, so it reports any unrelated ROS
    # node the developer happens to be running. These tests are about the
    # teardown order, not about the machine being idle.
    runner._verify_clean = lambda: None
    return runner


def test_teardown_stops_every_launch_even_when_an_earlier_step_throws(tmp_path):
    launches = {name: _sleeper(name, tmp_path) for name in SHUTDOWN_ORDER}
    runner = _runner_with()

    runner._teardown(launches, _ExplodingFriction())

    for name, launch in launches.items():
        assert not launch.alive(), f'{name} survived the teardown'
    assert run_benchmark._LIVE_LAUNCHES == []


def test_teardown_stops_launches_in_order(tmp_path):
    order = []

    class _Recording:
        def __init__(self, name):
            self.name = name

        def shutdown(self, timeout_s=180.0):
            order.append(self.name)

    launches = {name: _Recording(name) for name in reversed(SHUTDOWN_ORDER)}
    _runner_with()._teardown(launches, None)

    # Identification before the controller, publisher last.
    assert order == list(SHUTDOWN_ORDER)
    assert order[0] == 'on_track_sys_id'
    assert order[-1] == 'raceline_publisher'


def test_teardown_reports_a_failing_bridge_without_skipping_the_launch_shutdown(tmp_path, capsys):
    class _BrokenBridge:
        def ensure_unconfigured(self):
            raise RuntimeError('lifecycle service gone')

        def state_name(self):
            raise RuntimeError('lifecycle service gone')

    launch = _sleeper('on_track_sys_id', tmp_path)
    runner = _runner_with(_BrokenBridge())
    runner._teardown({'on_track_sys_id': launch}, None)

    assert not launch.alive()
    assert 'WARNING' in capsys.readouterr().out


# ---------------- the orphan guard ----------------

def test_stop_orphans_kills_anything_still_registered(tmp_path):
    launch = _sleeper('adaptive_stack', tmp_path)
    assert launch.alive()

    run_benchmark._stop_orphans()

    assert not launch.alive()
    assert run_benchmark._LIVE_LAUNCHES == []


def test_launch_children_do_not_share_the_runners_process_group(tmp_path):
    """Ctrl-C in the terminal must not reach them - the runner decides the order."""
    launch = _sleeper('adaptive_stack', tmp_path)
    assert os.getpgid(launch.proc.pid) != os.getpgid(0)


def test_shutdown_escalates_to_sigkill_when_a_launch_ignores_sigint(tmp_path):
    launch = Launch('stubborn', 'unused', tmp_path / 'stubborn.log')
    launch.command = "exec bash -c 'trap \"\" INT; sleep 600'"
    launch.start()
    time.sleep(0.5)

    launch.shutdown(timeout_s=2.0)

    assert not launch.alive()
    assert run_benchmark._LIVE_LAUNCHES == []


def test_shutdown_is_idempotent(tmp_path):
    launch = _sleeper('raceline_publisher', tmp_path)
    launch.shutdown(timeout_s=5.0)
    launch.shutdown(timeout_s=5.0)
    assert not launch.alive()


def test_shutdown_of_a_never_started_launch_is_a_noop(tmp_path):
    Launch('unstarted', 'unused', tmp_path / 'x.log').shutdown()
    assert run_benchmark._LIVE_LAUNCHES == []


def test_sleeper_is_actually_reaped(tmp_path):
    """Guards the test helper itself: a zombie would make every assertion above pass."""
    launch = _sleeper('raceline_publisher', tmp_path)
    pid = launch.proc.pid
    launch.shutdown(timeout_s=5.0)
    with pytest.raises((ProcessLookupError, subprocess.SubprocessError)):
        os.kill(pid, 0)
