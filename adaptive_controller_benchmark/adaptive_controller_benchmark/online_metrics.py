"""Pure-logic metrics for benchmarking adaptive_controller_manager's PP/MPC switching FSM.

State-name strings fed to SwitchTracker must match adaptive_controller_manager's
manager_node.cpp::fsmStateName() literally: BOOTSTRAP_PP, RUNNING_PP, SWITCHING_TO_MPC,
RUNNING_MPC, SWITCHING_TO_PP, EMERGENCY_HALT. This is a string-literal coupling (the
manager publishes std_msgs/String on manager/state), not a shared enum - if the manager's
FSM state names ever change, update STATE_COLORS below and the state-name comparisons in
adaptive_controller_benchmark_node.py to match.
"""
import math

import numpy as np

# Mirrors manager_node.cpp::activeControllerLabel()'s color scheme (also used by the
# manager/active_controller_marker RViz marker) so plots stay visually consistent with
# what's shown in RViz during a run.
STATE_COLORS = {
    'BOOTSTRAP_PP': '#999999',
    'RUNNING_PP': '#00b300',
    'SWITCHING_TO_MPC': '#ff9900',
    'RUNNING_MPC': '#0066ff',
    'SWITCHING_TO_PP': '#ff9900',
    'EMERGENCY_HALT': '#ff0000',
}


class SignalStats:
    """Streaming RMS / max-abs / ITAE accumulator for a single scalar signal.

    Not a ground-truth-vs-estimate comparison (see tire_force_benchmark.online_metrics.
    OnlineBenchmark for that shape) - this tracks absolute tracking-error magnitude, the
    metric shape Demeter et al. 2025 ("The Autonomous Software Stack of the FRED-003C",
    arXiv:2504.18439) use in their Table II to compare Pure Pursuit / Stanley / a
    combined switching controller.
    """

    def __init__(self, name: str = 'signal'):
        self.name = name
        self.reset()

    def reset(self):
        self._n = 0
        self._sum_sq = 0.0
        self._max_abs = 0.0
        self._itae = 0.0

    def update(self, value: float, dt: float, t_since_reset: float):
        """dt: seconds since the previous update (ITAE's time-step).
        t_since_reset: seconds since this accumulator's time origin (run start or lap
        start, caller's choice) - the 't' in ITAE = integral of t*|e(t)| dt.
        """
        self._n += 1
        self._sum_sq += value * value
        self._max_abs = max(self._max_abs, abs(value))
        if dt > 0.0:
            self._itae += dt * t_since_reset * abs(value)

    def metrics(self) -> dict:
        if self._n == 0:
            return {'n': 0, 'rms': 0.0, 'max_abs': 0.0, 'itae': 0.0}
        return {
            'n': self._n,
            'rms': math.sqrt(self._sum_sq / self._n),
            'max_abs': self._max_abs,
            'itae': self._itae,
        }


class HandoverHistoryBuffer:
    """Bounded-memory time-history for plotting: (t, state, e_y, heading_error, v_x,
    steering, speed_cmd) tuples. Same logarithmic-decimation strategy as
    tire_force_benchmark.online_metrics.HistoryBuffer (halve + double stride once
    2*max_points samples seen) so memory stays bounded on long runs while still
    spanning the full session.
    """

    def __init__(self, max_points: int = 5000):
        self.max_points = max(100, int(max_points))
        self._stride = 1
        self._count = 0
        self.t = []
        self.state = []
        self.e_y = []
        self.heading_error = []
        self.v_x = []
        self.steering = []
        self.speed_cmd = []
        self.solve_time_ms = []
        self.pos_x = []
        self.pos_y = []
        self.lap_idx = []

    def add(self, t, state, e_y, heading_error, v_x, steering, speed_cmd, solve_time_ms=0.0,
            pos_x=0.0, pos_y=0.0, lap_idx=0):
        self._count += 1
        if (self._count - 1) % self._stride != 0:
            return
        self.t.append(t)
        self.state.append(state)
        self.e_y.append(e_y)
        self.heading_error.append(heading_error)
        self.v_x.append(v_x)
        self.steering.append(steering)
        self.speed_cmd.append(speed_cmd)
        self.solve_time_ms.append(solve_time_ms)
        self.pos_x.append(pos_x)
        self.pos_y.append(pos_y)
        self.lap_idx.append(lap_idx)
        if len(self.t) > 2 * self.max_points:
            self.t = self.t[::2]
            self.state = self.state[::2]
            self.e_y = self.e_y[::2]
            self.heading_error = self.heading_error[::2]
            self.v_x = self.v_x[::2]
            self.steering = self.steering[::2]
            self.speed_cmd = self.speed_cmd[::2]
            self.solve_time_ms = self.solve_time_ms[::2]
            self.pos_x = self.pos_x[::2]
            self.pos_y = self.pos_y[::2]
            self.lap_idx = self.lap_idx[::2]
            self._stride *= 2


class SwitchTracker:
    """Tracks FSM dwell time, transition counts, and per-switch-episode handover
    transients from one (timestamp, state, v_cmd, steering) sample per control tick.

    Grounded in Hespanha & Morse, "Switching between stabilizing controllers,"
    Automatica 38(11), pp. 1905-1917, 2002 - dwell time (how long the FSM stays in a
    state before switching again) is the standard way that literature characterizes
    switched-system behavior; a controller that chatters between states with very short
    dwell times is a red flag even when steady-state tracking error looks fine.
    """

    SWITCHING_STATES = ('SWITCHING_TO_MPC', 'SWITCHING_TO_PP')

    def __init__(self):
        self.dwell_times = {}          # state -> list[seconds]
        self.transition_counts = {}    # (from_state, to_state) -> int
        self.episodes = []             # list of dict, see _open_episode()
        self._current_state = None
        self._state_entered_at = None
        self._episode = None
        self._last_steering = 0.0

    def _open_episode(self, to_state: str, entry_steering: float):
        return {
            'to_state': to_state,
            'samples': [],              # (t_since_switch_start, v_cmd, steering)
            'exit_steering': self._last_steering,
            'entry_steering': entry_steering,
        }

    def update(self, t: float, state: str, v_cmd: float, steering: float, extra: dict = None):
        """extra: optional dict of additional per-tick values (e.g. raw pp/mpc steering)
        recorded alongside each switching-episode sample, for richer handover plots -
        not used by dwell-time/transition-count bookkeeping.
        """
        if self._current_state is None:
            self._current_state = state
            self._state_entered_at = t
        elif state != self._current_state:
            dwell = t - self._state_entered_at
            self.dwell_times.setdefault(self._current_state, []).append(dwell)
            key = (self._current_state, state)
            self.transition_counts[key] = self.transition_counts.get(key, 0) + 1

            if self._episode is not None:
                self._episode['exit_steering'] = self._last_steering
                self.episodes.append(self._episode)
                self._episode = None

            if state in self.SWITCHING_STATES:
                self._episode = self._open_episode(state, steering)

            self._current_state = state
            self._state_entered_at = t

        if self._episode is not None:
            self._episode['samples'].append((t - self._state_entered_at, v_cmd, steering, extra))

        self._last_steering = steering

    def finalize(self, t_end: float):
        """Close out whatever dwell interval/episode is still open - call once at shutdown."""
        if self._current_state is not None and self._state_entered_at is not None:
            self.dwell_times.setdefault(self._current_state, []).append(t_end - self._state_entered_at)
        if self._episode is not None:
            self.episodes.append(self._episode)
            self._episode = None

    def summary(self) -> dict:
        dwell_summary = {}
        for state, vals in self.dwell_times.items():
            dwell_summary[state] = {
                'n': len(vals),
                'mean': sum(vals) / len(vals) if vals else 0.0,
                'max': max(vals) if vals else 0.0,
            }
        return {
            'dwell': dwell_summary,
            'transitions': dict(self.transition_counts),
            'n_episodes': len(self.episodes),
        }


class LapTracker:
    """Derives lap completion from nearest-waypoint arc-length (s_m) wraparound against
    /raceline_waypoints. No lap-completion topic exists anywhere in this repo, so this
    re-derives it standalone - the same "re-derive rather than assume a topic exists"
    approach tire_force_benchmark already uses for its own vehicle-state-prediction
    physics (see docs/tire_force_benchmark.md).
    """

    def __init__(self):
        self._x = None
        self._y = None
        self._s = None
        self._track_length = 0.0
        self._last_s = None
        self._lap_start_t = None
        self.lap_times = []
        self.current_lap = 0

    def set_waypoints(self, x_m, y_m, s_m):
        self._x = np.asarray(x_m, dtype=float)
        self._y = np.asarray(y_m, dtype=float)
        self._s = np.asarray(s_m, dtype=float)
        self._track_length = float(self._s.max()) if self._s.size else 0.0

    def get_waypoints_xy(self):
        return self._x, self._y

    def current_lap_index(self) -> int:
        return self.current_lap

    def ready(self) -> bool:
        return self._x is not None and self._x.size > 0

    def track_length(self) -> float:
        return self._track_length

    def last_s(self):
        """Arc-length of the last sample, or None before the first update."""
        return self._last_s

    def nearest_s(self, x: float, y: float) -> float:
        dx = self._x - x
        dy = self._y - y
        idx = int(np.argmin(dx * dx + dy * dy))
        return float(self._s[idx])

    def update(self, t: float, x: float, y: float):
        if not self.ready():
            return
        s = self.nearest_s(x, y)
        # A lap boundary is a large *decrease* in arc-length (wrapping from near the
        # track length back to ~0) - anything less than half the track length is
        # ordinary forward progress, never a wraparound.
        #
        # Everything before the FIRST wraparound is an out-lap (lap index 0, not
        # timed): the car spawns wherever the simulator puts it, which in this
        # repo is behind the raceline's s=0, so timing from the first sample
        # reports the few seconds it takes to reach the start line as lap 1.
        if self._last_s is not None and self._track_length > 0.0:
            if (self._last_s - s) > 0.5 * self._track_length:
                if self._lap_start_t is not None:
                    self.lap_times.append(t - self._lap_start_t)
                self._lap_start_t = t
                self.current_lap += 1
        self._last_s = s
