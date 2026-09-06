"""Fail a scenario when the car actually crashes, and record every contact.

The bridge publishes `/sim/feedback/collision` (`carla_msgs/CarlaCollisionEvent`)
from CARLA's `sensor.other.collision`, one message per contact frame. A racing
line that brushes a barrier is not a failed run, so the first event alone is not
enough: the run fails on the first event whose HORIZONTAL impulse exceeds
`collision_impulse_threshold`.

Horizontal, not the 3D magnitude, because the road reports contact too. The car
spawns slightly above the surface and drops onto it, which fires a burst of
events carrying the whole vehicle weight in z - measured at spawn: 267.98 N*s of
which 267.96 is vertical, against 3.67 N*s of horizontal. Thresholding the 3D
magnitude fails every run on the settle. A crash into a barrier or another car is
lateral, so `sqrt(x^2 + y^2)` separates the two cleanly. `other_actor_id` does
not: static geometry reports 0, and that covers barriers as well as the road.

Impulse is in N*s, i.e. mass times the velocity change CARLA resolved for the
contact. On the 240 kg vehicle of the bridge config, 200 N*s is about a 0.83 m/s
knock - a hit, not a graze. Every collision is written to `collisions.csv` with
both magnitudes, so the threshold can be calibrated against a real run.
"""
import csv
import math
import threading

from carla_msgs.msg import CarlaCollisionEvent
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

# Matches the bridge's publisher: collisions are rare and bursty, and dropping
# the one that ended the run would be the whole point missed.
QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)


class CollisionMonitor(Node):

    def __init__(self, threshold, csv_path, topic='/sim/feedback/collision'):
        super().__init__('benchmark_collision_monitor')
        self.threshold = float(threshold)
        self._crash = None
        self._count = 0
        self._lock = threading.Lock()

        self._csv_file = open(csv_path, 'w', newline='', encoding='utf-8')
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(['t_s', 'other_actor_id', 'impulse_x', 'impulse_y',
                            'impulse_z', 'impulse_magnitude',
                            'impulse_horizontal', 'over_threshold'])

        self.create_subscription(CarlaCollisionEvent, topic, self._on_event, QOS)

    def _on_event(self, msg):
        impulse = msg.normal_impulse
        magnitude = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        horizontal = math.hypot(impulse.x, impulse.y)
        over = horizontal >= self.threshold
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._csv.writerow([f'{t:.3f}', msg.other_actor_id, f'{impulse.x:.4f}',
                            f'{impulse.y:.4f}', f'{impulse.z:.4f}',
                            f'{magnitude:.4f}', f'{horizontal:.4f}', int(over)])
        with self._lock:
            self._count += 1
            # Latch the first one over the line: the contact burst that follows
            # is the same crash, and the run is already over.
            if over and self._crash is None:
                self._crash = (
                    f'collision with actor {msg.other_actor_id}: horizontal impulse '
                    f'{horizontal:.1f} N*s exceeds the {self.threshold:.1f} N*s threshold')

    def crash(self):
        """Description of the first over-threshold collision, or None."""
        with self._lock:
            return self._crash

    def count(self):
        with self._lock:
            return self._count

    def close(self):
        self._csv_file.flush()
        self._csv_file.close()
