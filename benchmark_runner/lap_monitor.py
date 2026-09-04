"""Counts laps for the runner, using the benchmark's own lap definition.

Nothing in this workspace publishes a lap count, so `adaptive_controller_benchmark`
derives it from `s_m` wraparound on `/raceline_waypoints` (`LapTracker`). This
node reuses that exact class rather than re-deriving it, so "6 laps" means the
same thing to the runner as it does to `lap_times.png`.

Out-lap: the car spawns BEHIND the raceline's `s = 0`, so the drive to the start
line is not a lap. `LapTracker` encodes that - `current_lap` counts wraparounds,
everything before the first one is `lap_idx = 0` and untimed, and `lap_times`
only gains an entry on the second wraparound. `wait_for_laps(n)` therefore waits
on `len(lap_times) >= n`, i.e. n + 1 wraparounds; waiting on `current_lap >= n`
would stop a full lap early and leave the last lap incomplete in the plots.
"""
import threading
import time

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from f1tenth_msgs.msg import WaypointArray

from adaptive_controller_benchmark.online_metrics import LapTracker


class LapMonitor(Node):

    def __init__(self, odom_topic='/odom', waypoint_topic='/raceline_waypoints',
                 on_lap=None, on_progress=None):
        super().__init__('benchmark_lap_monitor')
        self.lap_tracker = LapTracker()
        self._on_lap = on_lap
        self._on_progress = on_progress
        self._start_t = None
        self._last_completed = 0
        self._waypoints = None
        self._lock = threading.Lock()

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self.create_subscription(WaypointArray, waypoint_topic, self._waypoint_cb, latched_qos)

    def reset(self):
        """Drop the previous scenario's laps, keeping the waypoints already received.

        The runner reuses one monitor for the whole sweep, so without this the
        next scenario starts with `lap_times` already full and `wait_for_laps`
        returns immediately with the previous scenario's times.
        """
        with self._lock:
            self.lap_tracker = LapTracker()
            if self._waypoints is not None:
                self.lap_tracker.set_waypoints(*self._waypoints)
            self._start_t = None
            self._last_completed = 0

    # ---------------- state ----------------

    def completed_laps(self):
        with self._lock:
            return len(self.lap_tracker.lap_times)

    def wraparounds(self):
        with self._lock:
            return self.lap_tracker.current_lap

    def lap_times(self):
        with self._lock:
            return list(self.lap_tracker.lap_times)

    def lap_fraction(self):
        """How far into the current lap the car is, 0..1 by arc length.

        Used by the `step40_mid_lap1` friction schedule. Returns None until
        waypoints have arrived and the car has produced a position sample.
        """
        with self._lock:
            last_s = self.lap_tracker.last_s() if self.lap_tracker.ready() else None
            length = self.lap_tracker.track_length()
            if last_s is None or length <= 0.0:
                return None
            return float(last_s) / length

    def ready(self):
        with self._lock:
            return self.lap_tracker.ready()

    # ---------------- callbacks ----------------

    def _waypoint_cb(self, msg):
        with self._lock:
            # Kept so `reset` can re-apply them: the raceline is the same for
            # every scenario and is not guaranteed to be redelivered.
            self._waypoints = (
                [w.x_m for w in msg.waypoints],
                [w.y_m for w in msg.waypoints],
                [w.s_m for w in msg.waypoints])
            self.lap_tracker.set_waypoints(*self._waypoints)

    def _odom_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            if not self.lap_tracker.ready():
                return
            if self._start_t is None:
                self._start_t = t
            before = self.lap_tracker.current_lap
            self.lap_tracker.update(
                t - self._start_t, msg.pose.pose.position.x, msg.pose.pose.position.y)
            after = self.lap_tracker.current_lap
            completed = len(self.lap_tracker.lap_times)
            crossed = after > before
            newly_completed = completed > self._last_completed
            self._last_completed = completed

        if crossed and self._on_lap is not None:
            self._on_lap(after, completed)
        if newly_completed and self._on_progress is not None:
            self._on_progress(completed)

    # ---------------- waiting ----------------

    def wait_for_laps(self, n_laps, timeout_s, stall_check=None, poll_s=0.5):
        """Block until `n_laps` TIMED laps are complete.

        `stall_check`, if given, is called each poll and may raise to abort the
        scenario early (the runner uses it to notice a dead launch process).
        Returns True on success, False on timeout.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if stall_check is not None:
                stall_check()
            if self.completed_laps() >= n_laps:
                return True
            time.sleep(poll_s)
        return self.completed_laps() >= n_laps
