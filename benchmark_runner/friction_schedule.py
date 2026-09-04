"""Drives the plant's tire friction over a run, and records what it commanded.

The bridge exposes `/sim/control/tire_friction` (`std_msgs/Float32`), applied via
`ApplyPhysicsControl` with the rigid-body velocity sampled and written straight
back, so grip can change while the car is driving. That is the mechanism
paper/sections/experiments.tex SVI-I specifies; nothing published to it before.

Schedules follow LLA-MPC, as the paper does:

  constant          hold the configured value
  decay_2pct_s      mu(t) = mu0 * (1 - 0.02 t), from the first command
  step40_end_lap1   0.6 * mu0 once the first TIMED lap completes
  step40_mid_lap1   0.6 * mu0 halfway through the first timed lap, by arc length

Lap indices are the runner's: the out-lap (spawn to the first s=0 crossing) never
triggers a step, so "end of lap 1" is the second `s_m` wraparound.

The commanded value is the configured per-wheel coefficient. PhysX multiplies it
by the road surface factor (0.70 on silverstone), so the effective mu the physics
step uses is lower - that effective value comes back on
`/sim/feedback/tire_forces.tire_friction` and is what the benchmark scores
against. `mu_commanded.csv` records the command, not the effective value, and
says so in its header.
"""
import csv
import math
import threading
import time

from rclpy.node import Node
from std_msgs.msg import Float32

SCHEDULES = ('constant', 'decay_2pct_s', 'step40_end_lap1', 'step40_mid_lap1')

DECAY_PER_S = 0.02
STEP_FACTOR = 0.6
MU_FLOOR = 0.05          # PhysX needs a positive coefficient; a decaying run is
                         # meaningless long before this, but it must not reach 0.
PUBLISH_HZ = 30.0        # the sim's own step: fixed_delta_seconds 0.033


class FrictionSchedule(Node):

    def __init__(self, schedule, nominal_mu, csv_path, lap_monitor=None,
                 topic='/sim/control/tire_friction'):
        super().__init__('benchmark_friction_schedule')
        if schedule not in SCHEDULES:
            raise ValueError(f'unknown friction_schedule {schedule!r}, expected one of {SCHEDULES}')
        self.schedule = schedule
        self.nominal_mu = float(nominal_mu)
        self.lap_monitor = lap_monitor
        self._pub = self.create_publisher(Float32, topic, 10)
        self._t0 = None
        self._stepped = False
        self._lock = threading.Lock()

        self._csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(['t_s', 'commanded_tire_friction', 'schedule'])

        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            t = now - self._t0
        mu = self._mu_at(t)
        self._pub.publish(Float32(data=float(mu)))
        self._csv.writerow([f'{t:.3f}', f'{mu:.6f}', self.schedule])

    def _mu_at(self, t):
        if self.schedule == 'constant':
            return self.nominal_mu
        if self.schedule == 'decay_2pct_s':
            return max(MU_FLOOR, self.nominal_mu * (1.0 - DECAY_PER_S * t))
        if self._stepped or self._step_due():
            self._stepped = True
            return STEP_FACTOR * self.nominal_mu
        return self.nominal_mu

    def _step_due(self):
        if self.lap_monitor is None:
            return False
        if self.schedule == 'step40_end_lap1':
            return self.lap_monitor.completed_laps() >= 1
        if self.schedule == 'step40_mid_lap1':
            # Only inside the first timed lap: wraparounds == 1 means the
            # out-lap is over and lap 1 is under way.
            if self.lap_monitor.wraparounds() != 1:
                return self.lap_monitor.completed_laps() >= 1
            fraction = self.lap_monitor.lap_fraction()
            return fraction is not None and fraction >= 0.5
        return False

    def restore_nominal(self):
        """Put the surface back before the next scenario configures the bridge."""
        self._timer.cancel()
        msg = Float32(data=float(self.nominal_mu))
        for _ in range(5):
            self._pub.publish(msg)
            time.sleep(0.05)
        self._csv.writerow([f'{math.nan}', f'{self.nominal_mu:.6f}', 'restore'])

    def close(self):
        self._csv_file.flush()
        self._csv_file.close()
