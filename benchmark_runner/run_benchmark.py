#!/usr/bin/env python3
"""Run a sweep of identification + control scenarios and export their figures.

One scenario is: configure/activate the CARLA bridge, publish the raceline,
launch On-Track-SysID (with tire_force_benchmark), launch the adaptive control
stack (with adaptive_controller_benchmark), drive N timed laps, shut both stacks
down so their benchmark nodes export on `destroy_node()`, then deactivate and
clean up the bridge. Repeat for the next parameter set.

The CARLA server and the bridge PROCESS are the user's to start and stop:

    cd /home/ebrahim/Carla_ASU_Bridge && make launch_carla_sim AUTO_START=false

This runner only drives the bridge's lifecycle. Usage:

    make run_benchmark_scenarios
    make run_benchmark_scenarios SCENARIOS=benchmark_runner/scenarios.yaml
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bridge_config import BRIDGE_CONFIG, restore, set_tire_friction  # noqa: E402
from friction_schedule import FrictionSchedule  # noqa: E402
from lap_monitor import LapMonitor  # noqa: E402
from run_status import write_failure  # noqa: E402
from sim_control import BridgeLifecycle, SimNotRunning, wait_for_publisher  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CARLA_BRIDGE_SETUP = Path('/home/ebrahim/Carla_ASU_Bridge/install/ros_apps/setup.bash')

# Everything a scenario can leave behind. Checked after each teardown so
# scenario N+1 never inherits a stray node from scenario N.
PROC_PATTERNS = (
    'on_track_sys_id.py',
    'tire_force_benchmark_node',
    'adaptive_controller_benchmark_node',
    'adaptive_controller_manager_node',
    'mpc_path_tracking/mpc_node',
    'pure_pursuit_node.py',
    'raceline_publisher',
)

# Identification goes down BEFORE the controller: its last accepted parameter set
# must already have been forwarded and scored. The raceline publisher goes last so
# nothing loses its reference mid-shutdown.
SHUTDOWN_ORDER = ('on_track_sys_id', 'adaptive_stack', 'raceline_publisher')


def log(msg):
    print(f'[{time.strftime("%H:%M:%S")}] {msg}', flush=True)


class ScenarioFailed(RuntimeError):
    pass


class Aborted(ScenarioFailed):
    """The operator interrupted the run; tear the scenario down normally."""


# Ctrl-C must NOT tear the ROS context down under us: the teardown itself needs a
# live context (it publishes the nominal friction back and calls the bridge's
# lifecycle services), and it must reach the launches so their benchmark nodes
# get their graceful SIGINT and export. So rclpy's own signal handlers are
# disabled (see main) and this flag is what stops the lap wait instead.
_INTERRUPTED = threading.Event()

# Last-ditch orphan guard: children are started in their own session so the
# terminal's Ctrl-C does not reach them out of order, which means an unexpected
# exception in the runner would otherwise leave them driving the car.
_LIVE_LAUNCHES = []


def _install_signal_handlers():
    def handler(signum, _frame):
        if _INTERRUPTED.is_set():
            log('second interrupt - abandoning the run. Figures may be missing.')
            signal.signal(signal.SIGINT, signal.SIG_DFL)
            raise KeyboardInterrupt
        _INTERRUPTED.set()
        log('interrupt received - stopping this scenario cleanly; the benchmark exports '
            'take ~15 s. Ctrl-C again to abandon it.')

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def _stop_orphans():
    for launch in list(_LIVE_LAUNCHES):
        log(f'orphan guard: {launch.name} is still running')
        try:
            launch.shutdown(timeout_s=60.0)
        except Exception as exc:  # noqa: BLE001 - last line of defence
            log(f'orphan guard: could not stop {launch.name}: {exc}')


class Launch:
    """One `ros2 launch` child, shut down by SIGINT so its nodes export.

    Both benchmark nodes write their figures from `destroy_node()`, which only
    runs on a graceful shutdown - so this never escalates to SIGKILL on its own.
    `tire_force_benchmark` additionally ignores SIGINT/SIGTERM while exporting,
    and its launch file allows 90 s for it, so waiting is the whole contract.
    """

    def __init__(self, name, ros_args, log_path, env=None, source_bridge=False):
        self.name = name
        self.log_path = Path(log_path)
        self.env = env
        self.proc = None
        sources = ['/opt/ros/humble/setup.bash', str(REPO_ROOT / 'install' / 'setup.bash')]
        if source_bridge:
            # sim_manager_msgs/TireForces lives in the bridge workspace and must
            # be sourced AFTER this workspace's overlay (see the makefile).
            sources.append(str(CARLA_BRIDGE_SETUP))
        source_cmd = ' && '.join(f'source {s}' for s in sources)
        # `exec` so this Popen's pid IS ros2 launch's - SIGINT must reach it,
        # not an intermediate shell.
        self.command = f'{source_cmd} && exec ros2 launch {ros_args}'

    def start(self):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log(f'launching {self.name} (log: {self.log_path})')
        with self.log_path.open('w') as handle:
            self.proc = subprocess.Popen(
                ['bash', '-c', self.command],
                stdout=handle, stderr=subprocess.STDOUT,
                env=self.env, start_new_session=True, cwd=str(REPO_ROOT))
        _LIVE_LAUNCHES.append(self)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def shutdown(self, timeout_s=180.0):
        try:
            if self.proc is None or self.proc.poll() is not None:
                return
            log(f'stopping {self.name} (SIGINT, waiting up to {timeout_s:.0f}s for its export)')
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=timeout_s)
                log(f'{self.name} exited cleanly')
            except subprocess.TimeoutExpired:
                log(f'WARNING: {self.name} did not exit within {timeout_s:.0f}s - killing its '
                    'group. Its figures may be missing.')
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                self.proc.wait(timeout=30)
        finally:
            if self in _LIVE_LAUNCHES:
                _LIVE_LAUNCHES.remove(self)


class BenchmarkRunner:

    # Snapshot of the bridge config, taken in __init__ before anything patches
    # it. None means no snapshot exists, so there is nothing to restore.
    bridge_config_original = None

    def __init__(self, config):
        self.config = config
        self.n_laps = int(config['n_laps'])
        self.lap_timeout_s = float(config.get('lap_timeout_s', 1200))
        self.graphs_root = REPO_ROOT / config.get('graphs_root', 'graphs')
        self.raceline_csv = self._resolve(config['raceline_csv'])
        self.psi_offset_rad = config.get('psi_offset_rad', 0.0)
        self.nominal_mu = float(config.get('nominal_tire_friction', 1.5))
        # Snapshot once for the whole sweep, before anything is patched, so a
        # scenario that dies mid-teardown cannot bake its own value in as the
        # value everything after it restores to.
        self.bridge_config_original = BRIDGE_CONFIG.read_text(encoding='utf-8')

        self.node = Node('benchmark_runner')
        self.lap_monitor = LapMonitor(on_progress=self._on_lap_complete)
        self.bridge = BridgeLifecycle(self.node)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.lap_monitor)
        self._spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin_thread.start()

    def _resolve(self, path):
        candidate = Path(path)
        return candidate if candidate.is_absolute() else (REPO_ROOT / candidate)

    def _on_lap_complete(self, completed):
        log(f'  lap {completed}/{self.n_laps} complete')

    # ---------------- one scenario ----------------

    def run_scenario(self, scenario):
        name = scenario['name']
        log(f'===== scenario {name} =====')
        # One monitor serves the whole sweep, so its lap count must start at 0.
        self.lap_monitor.reset()
        ident_dir = self.graphs_root / 'identification' / name
        control_dir = self.graphs_root / 'control' / name
        for directory in (ident_dir, control_dir):
            self._wipe(directory)
            (directory / 'raw').mkdir(parents=True, exist_ok=True)

        log_dir = control_dir / 'raw'
        launches = {}
        friction = None
        try:
            # Before configure: that is when the bridge reads the file.
            mu = float(scenario.get('friction_value_const', self.nominal_mu))
            log(f'  tire friction:  {mu} configured '
                f'(~{mu * 0.70:.2f} effective after the road factor)')
            set_tire_friction(mu)
            self._activate_bridge()

            launches['raceline_publisher'] = Launch(
                'raceline_publisher',
                f'raceline_publisher raceline_publisher.launch.py '
                f'raceline_csv:={self.raceline_csv} '
                f'psi_offset_rad:={self.psi_offset_rad}',
                log_dir / 'raceline_publisher.log')
            launches['raceline_publisher'].start()
            if not wait_for_publisher(self.node, '/raceline_waypoints', 60.0):
                raise ScenarioFailed('/raceline_waypoints never appeared')

            # Identification comes up before the car can move, so it observes
            # the whole run rather than joining it partway.
            launches['on_track_sys_id'] = Launch(
                'on_track_sys_id',
                f'on_track_sys_id sys_id.launch.py '
                f'enable_tire_force_benchmark:=true '
                f'benchmark_update_params_enable:=true '
                f'racecar_version:={self.config.get("racecar_version", "SIM")} '
                f'tire_force_benchmark_plot_output_dir:={ident_dir} '
                f'tire_force_benchmark_csv_output_path:={ident_dir / "raw" / "tire_forces.csv"}',
                ident_dir / 'raw' / 'on_track_sys_id.log',
                env=self._sysid_env(scenario), source_bridge=True)
            launches['on_track_sys_id'].start()

            # The manager is the sole /drive writer, so the car starts moving here.
            launches['adaptive_stack'] = Launch(
                'adaptive_stack',
                f'adaptive_controller_manager adaptive_stack.launch.py '
                f'enable_controller_benchmark:=true '
                f'controller_benchmark_plot_output_dir:={control_dir} '
                f'controller_benchmark_csv_output_path:={control_dir / "raw" / "controller.csv"}',
                log_dir / 'adaptive_stack.log')
            launches['adaptive_stack'].start()

            # The same mu the config was patched with: this publishes at 30 Hz
            # and would otherwise overwrite the wheels with the sweep default
            # one tick after the bridge came up.
            friction = FrictionSchedule(
                scenario.get('friction_schedule', 'constant'), mu,
                ident_dir / 'raw' / 'mu_commanded.csv', lap_monitor=self.lap_monitor)
            self.executor.add_node(friction)

            log(f'driving {self.n_laps} timed laps (plus the out-lap from the spawn point)')
            reached = self.lap_monitor.wait_for_laps(
                self.n_laps, self.lap_timeout_s,
                stall_check=lambda: self._require_alive(launches))
            if not reached:
                raise ScenarioFailed(
                    f'only {self.lap_monitor.completed_laps()}/{self.n_laps} timed laps in '
                    f'{self.lap_timeout_s:.0f}s')
            log(f'lap times: {[round(t, 2) for t in self.lap_monitor.lap_times()]}')
            return True
        except (ScenarioFailed, SimNotRunning) as exc:
            log(f'SCENARIO FAILED ({name}): {exc}')
            # Written before the finally SIGINTs the stacks, so the benchmark
            # nodes can read it while exporting and mark where the data stops.
            self._record_failure((ident_dir, control_dir), str(exc))
            return False
        finally:
            self._teardown(launches, friction)

    def _teardown(self, launches, friction):
        """Every step isolated, because stopping the launches must happen no
        matter what: a skipped SIGINT leaves the identification and control
        stacks driving the car after the runner has exited."""
        self._safely('restoring nominal friction', self._stop_friction, friction)
        for key in SHUTDOWN_ORDER:
            if key in launches:
                self._safely(f'stopping {key}', launches[key].shutdown)
        self._safely('returning the bridge to unconfigured', self._deactivate_bridge)
        # After cleanup, so the write cannot race the bridge's own load_config.
        if self.bridge_config_original is not None:
            self._safely('restoring the bridge config', restore, self.bridge_config_original)
        self._verify_clean()

    def _record_failure(self, directories, reason):
        """Leave the reason where each benchmark node's plot export will find it.

        `_wipe` clears both directories at scenario start, so the file's mere
        presence means this run failed - a clean run leaves none.
        """
        for directory in directories:
            try:
                write_failure(directory, reason)
            except OSError as exc:  # noqa: BLE001 - never mask the real failure
                log(f'WARNING: could not record the failure in {directory}: {exc}')

    def _stop_friction(self, friction):
        if friction is None:
            return
        friction.restore_nominal()
        self.executor.remove_node(friction)
        friction.close()
        friction.destroy_node()

    def _safely(self, description, func, *args):
        try:
            func(*args)
        except Exception as exc:  # noqa: BLE001 - teardown must continue regardless
            log(f'WARNING: {description} failed: {exc}')

    def _sysid_env(self, scenario):
        env = dict(os.environ)
        env['SYSID_NN_PARAMS_FILE'] = str(self._resolve(scenario['nn_params_yaml']))
        env['SYSID_PACEJKA_PARAMS_FILE'] = str(self._resolve(scenario['pacejka_params_yaml']))
        env['MPLBACKEND'] = 'Agg'
        log(f'  nn params:      {env["SYSID_NN_PARAMS_FILE"]}')
        log(f'  pacejka params: {env["SYSID_PACEJKA_PARAMS_FILE"]}')
        return env

    def _require_alive(self, launches):
        if _INTERRUPTED.is_set():
            raise Aborted('interrupted by the operator')
        for launch in launches.values():
            if not launch.alive():
                raise ScenarioFailed(f'{launch.name} exited early - see {launch.log_path}')

    def _wipe(self, directory):
        """Overwrite means overwrite: no figure or CSV from an earlier run survives."""
        if directory.exists():
            shutil.rmtree(directory)
        directory.mkdir(parents=True, exist_ok=True)

    # ---------------- simulator ----------------

    def _activate_bridge(self):
        log('bridge: configure -> activate')
        self.bridge.ensure_unconfigured()
        if not self.bridge.transition('configure'):
            raise ScenarioFailed('bridge configure was rejected')
        if not self.bridge.transition('activate'):
            raise ScenarioFailed('bridge activate was rejected')
        for topic in ('/odom', '/sim/feedback/tire_forces'):
            if not wait_for_publisher(self.node, topic, 60.0):
                raise ScenarioFailed(f'{topic} never appeared after activate')
        log(f'bridge: {self.bridge.state_name()}')

    def _deactivate_bridge(self):
        try:
            log('bridge: deactivate -> cleanup')
            self.bridge.ensure_unconfigured()
            log(f'bridge: {self.bridge.state_name()}')
        except SimNotRunning as exc:
            log(f'WARNING: could not return the bridge to unconfigured: {exc}')

    def _verify_clean(self):
        survivors = []
        for pattern in PROC_PATTERNS:
            # Bracket the first character so pgrep does not match itself.
            result = subprocess.run(
                ['pgrep', '-af', f'[{pattern[0]}]{pattern[1:]}'],
                capture_output=True, text=True)
            if result.stdout.strip():
                survivors.append(result.stdout.strip())
        if survivors:
            raise RuntimeError(
                'processes survived teardown - refusing to start the next scenario:\n'
                + '\n'.join(survivors))
        log('teardown verified clean')

    def close(self):
        # Join the spin thread before destroying anything: a node torn down
        # while the executor is still inside spin() aborts the process during
        # interpreter teardown ("terminate called without an active exception").
        self.executor.shutdown()
        self._spin_thread.join(timeout=10.0)
        self.executor.remove_node(self.lap_monitor)
        self.executor.remove_node(self.node)
        self.lap_monitor.destroy_node()
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).resolve().parent / 'scenarios.yaml'))
    parser.add_argument('--only', action='append', default=None,
                        help='Run only the named scenario (repeatable)')
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    with config_path.open() as handle:
        config = yaml.safe_load(handle)

    scenarios = config['scenarios']
    if args.only:
        scenarios = [s for s in scenarios if s['name'] in args.only]
        missing = set(args.only) - {s['name'] for s in scenarios}
        if missing:
            parser.error(f'no such scenario(s) in {config_path}: {sorted(missing)}')
    if not scenarios:
        parser.error(f'no scenarios to run in {config_path}')

    # SignalHandlerOptions.NO: rclpy's default handler shuts the context down on
    # Ctrl-C, which would break the teardown that still has to publish and call
    # services - and, worse, throw out of the finally before the launches are
    # stopped, leaving the identification and control stacks running.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    _install_signal_handlers()
    runner = BenchmarkRunner(config)
    results = {}
    aborted = False
    try:
        runner.bridge.require_running()
        for scenario in scenarios:
            if _INTERRUPTED.is_set():
                log(f'interrupted - not starting {scenario["name"]}')
                aborted = True
                break
            results[scenario['name']] = runner.run_scenario(scenario)
        if _INTERRUPTED.is_set():
            aborted = True
    except SimNotRunning as exc:
        log(str(exc))
        return 2
    except RuntimeError as exc:
        # A teardown that left processes behind: stop rather than let the next
        # scenario inherit them.
        log(f'SWEEP ABORTED: {exc}')
        aborted = True
    except KeyboardInterrupt:
        log('abandoned - some nodes may not have exported')
        aborted = True
    finally:
        # Belt and braces: a sweep that dies before a scenario's own teardown
        # must still leave the bridge config as it found it.
        try:
            if runner.bridge_config_original is not None:
                restore(runner.bridge_config_original)
        except Exception as exc:  # noqa: BLE001 - never mask the real outcome
            log(f'WARNING: could not restore {BRIDGE_CONFIG}: {exc}')
        _stop_orphans()
        try:
            runner.close()
        except Exception as exc:  # noqa: BLE001 - never mask the real outcome
            log(f'WARNING: runner shutdown failed: {exc}')
        if rclpy.ok():
            rclpy.shutdown()

    log('===== sweep summary =====')
    for scenario in scenarios:
        name = scenario['name']
        log(f'  {name}: {"ok" if results.get(name) else "FAILED" if name in results else "not run"}')
    if not aborted and results and all(results.values()):
        log('now build the cross-scenario figures: make compare_benchmark_scenarios')
        return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
