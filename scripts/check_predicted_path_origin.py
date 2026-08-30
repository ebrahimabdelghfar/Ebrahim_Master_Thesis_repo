#!/usr/bin/env python3
"""Report where /mpc/predicted_path starts relative to the vehicle.

Prints the first predicted pose in the body frame of the latest /odom sample:
`ahead` is positive in front of base_link, `left` is positive to its left.
A negative `ahead` means the MPC horizon starts behind the car.
"""

import math

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__('predicted_path_origin_probe')
        self.odom = None
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom', self.on_odom, best_effort)
        self.create_subscription(Path, '/mpc/predicted_path', self.on_path, 1)

    def on_odom(self, msg):
        self.odom = msg

    def on_path(self, msg):
        if self.odom is None or not msg.poses:
            return
        o = self.odom.pose.pose
        psi = yaw_of(o.orientation)
        p = msg.poses[0].pose.position
        dx, dy = p.x - o.position.x, p.y - o.position.y
        ahead = dx * math.cos(psi) + dy * math.sin(psi)
        left = -dx * math.sin(psi) + dy * math.cos(psi)
        age = (self.get_clock().now().nanoseconds
               - rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds) * 1e-9
        self.get_logger().info(
            f'frame={msg.header.frame_id} odom_frame={self.odom.header.frame_id} '
            f'poses={len(msg.poses)} ahead={ahead:+.2f} m left={left:+.2f} m '
            f'path_age={age:+.3f} s v={self.odom.twist.twist.linear.x:.1f} m/s')


def main():
    rclpy.init()
    rclpy.spin(Probe())


if __name__ == '__main__':
    main()
