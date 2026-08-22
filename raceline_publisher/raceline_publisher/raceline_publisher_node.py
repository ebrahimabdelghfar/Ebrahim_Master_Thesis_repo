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
        # Added to every CSV psi_rad before publishing. The rest of the stack
        # (track_geometry_utils, mpc_path_tracking) assumes the ROS convention:
        # psi = atan2(dy, dx), zero along +x. Raw global_racetrajectory_optimization
        # output instead defines psi = 0 along +y, i.e. psi_ros = psi_csv + pi/2.
        # The bundled f1tenth_racetracks/* CSVs are already ROS convention, so this
        # defaults to 0.0; set it to 1.5707963268 for a raw TUM-convention raceline.
        self.declare_parameter('psi_offset_rad', 0.0)

        self.raceline_csv = self.get_parameter('raceline_csv').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_initial_pose_enabled = self.get_parameter('publish_initial_pose').get_parameter_value().bool_value
        self.psi_offset_rad = self.get_parameter('psi_offset_rad').get_parameter_value().double_value

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

    @staticmethod
    def wrap_to_pi(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # Compares each published psi_rad against the geometric tangent
    # atan2(dy, dx) of the polyline. A raceline in the wrong heading convention
    # is silently accepted by every downstream node - it only shows up much
    # later as a heading_error pinned near a constant (pi/2 for raw TUM output),
    # which permanently blocks adaptive_controller_manager's theta_max arming
    # gate. Warn loudly at load time instead.
    def check_heading_convention(self, waypoints):
        n = len(waypoints)
        if n < 2:
            return
        diffs = []
        for i in range(n):
            a = waypoints[i]
            b = waypoints[(i + 1) % n]
            tangent = math.atan2(b.y_m - a.y_m, b.x_m - a.x_m)
            diffs.append(self.wrap_to_pi(tangent - a.psi_rad))
        mean_diff = sum(diffs) / len(diffs)
        if abs(mean_diff) > 0.1:
            self.get_logger().error(
                f"Raceline heading convention mismatch: published psi_rad differs from the "
                f"path tangent by {mean_diff:+.4f} rad on average. Downstream nodes assume "
                f"psi = atan2(dy, dx). Set psi_offset_rad to {self.wrap_to_pi(mean_diff):+.7f} "
                f"(currently {self.psi_offset_rad:+.7f}) to correct it."
            )
        else:
            self.get_logger().info(
                f"Heading convention OK (mean |psi_rad - tangent| offset {mean_diff:+.4f} rad)")

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
                    wp.psi_rad = self.wrap_to_pi(float(row[3]) + self.psi_offset_rad)
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

        self.get_logger().info(
            f"Loaded {len(waypoints)} waypoints from {self.raceline_csv} "
            f"(psi_offset_rad={self.psi_offset_rad:+.7f})")
        self.check_heading_convention(waypoints)

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
