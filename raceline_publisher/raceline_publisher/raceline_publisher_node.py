#!/usr/bin/env python3
import csv
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from f1tenth_msgs.msg import Waypoint, WaypointArray


class RacelinePublisher(Node):
    def __init__(self):
        super().__init__('raceline_publisher')

        self.declare_parameter('raceline_csv', '')
        self.declare_parameter('waypoint_topic', '/raceline_waypoints')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_initial_pose', True)

        self.raceline_csv = self.get_parameter('raceline_csv').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_initial_pose_enabled = self.get_parameter('publish_initial_pose').get_parameter_value().bool_value

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.waypoint_pub = self.create_publisher(WaypointArray, self.waypoint_topic, latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.waypoint_topic + '_markers', latched_qos)

        self.initial_pose = None
        self.waypoints_msg = None
        self.markers_msg = None

        self.load_raceline()

        if self.publish_initial_pose_enabled:
            self.publish_count = 0
            self.max_publishes = 5
            self.spawn_timer = self.create_timer(0.5, self.publish_initial_pose)

    def load_raceline(self):
        if not self.raceline_csv:
            self.get_logger().error("raceline_csv parameter not set.")
            return

        waypoints = []
        try:
            with open(self.raceline_csv, 'r') as f:
                reader = csv.reader(f, delimiter=';')
                for row in reader:
                    if not row or row[0].startswith('#'):
                        continue

                    wp = Waypoint()
                    wp.s_m = float(row[0])
                    wp.x_m = float(row[1])
                    wp.y_m = float(row[2])
                    wp.psi_rad = float(row[3])
                    wp.kappa_radpm = float(row[4])
                    wp.vx_mps = float(row[5])
                    wp.ax_mps2 = float(row[6])
                    waypoints.append(wp)

                    if self.initial_pose is None:
                        self.initial_pose = (wp.x_m, wp.y_m, wp.psi_rad)
        except Exception as e:
            self.get_logger().error(f"Error reading CSV {self.raceline_csv}: {e}")
            return

        if not waypoints:
            self.get_logger().error(f"No waypoints loaded from {self.raceline_csv}")
            return

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {self.raceline_csv}")

        self.waypoints_msg = WaypointArray()
        self.waypoints_msg.header.frame_id = self.frame_id
        self.waypoints_msg.header.stamp = self.get_clock().now().to_msg()
        self.waypoints_msg.waypoints = waypoints

        self.markers_msg = MarkerArray()

        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = self.waypoints_msg.header.stamp
        line_marker.ns = "raceline"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.1
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.8

        for wp in waypoints:
            p = Point()
            p.x = wp.x_m
            p.y = wp.y_m
            p.z = 0.0
            line_marker.points.append(p)

        p = Point()
        p.x = waypoints[0].x_m
        p.y = waypoints[0].y_m
        p.z = 0.0
        line_marker.points.append(p)

        self.markers_msg.markers.append(line_marker)

        self.waypoint_pub.publish(self.waypoints_msg)
        self.marker_pub.publish(self.markers_msg)
        self.get_logger().info(f"Published latched waypoints and markers to {self.waypoint_topic}")

    def publish_initial_pose(self):
        if not self.initial_pose:
            self.spawn_timer.cancel()
            return

        if self.publish_count >= self.max_publishes:
            self.spawn_timer.cancel()
            self.get_logger().info("Finished publishing initial pose.")
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        x, y, yaw = self.initial_pose

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.pose_pub.publish(msg)
        self.publish_count += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RacelinePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
