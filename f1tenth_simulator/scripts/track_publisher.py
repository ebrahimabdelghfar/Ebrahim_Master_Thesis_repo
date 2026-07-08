#!/usr/bin/env python3
import os
import csv
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from f1tenth_msgs.msg import Waypoint, WaypointArray

class TrackPublisher(Node):
    def __init__(self):
        super().__init__('track_publisher')
        
        # Declare parameters
        self.declare_parameter('map_name', 'YasMarina')
        self.declare_parameter('racetracks_dir', '')
        self.declare_parameter('waypoint_topic', '/raceline_waypoints')
        
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.racetracks_dir = self.get_parameter('racetracks_dir').get_parameter_value().string_value
        self.waypoint_topic = self.get_parameter('waypoint_topic').get_parameter_value().string_value

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Latched QoS for waypoints and markers
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.waypoint_pub = self.create_publisher(WaypointArray, self.waypoint_topic, latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, self.waypoint_topic + '_markers', latched_qos)
        
        # Timers
        self.spawn_timer = self.create_timer(0.5, self.publish_initial_pose)
        self.publish_count = 0
        self.max_publishes = 5

        self.initial_pose = None
        self.waypoints_msg = None
        self.markers_msg = None
        
        self.load_track_data()

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw

    def load_track_data(self):
        if not self.racetracks_dir:
            self.get_logger().error("racetracks_dir parameter not set.")
            return

        track_dir = os.path.join(self.racetracks_dir, self.map_name)
        raceline_csv = os.path.join(track_dir, f"{self.map_name}_raceline.csv")
        centerline_csv = os.path.join(track_dir, f"{self.map_name}_centerline.csv")

        csv_file = None
        if os.path.exists(raceline_csv):
            csv_file = raceline_csv
        elif os.path.exists(centerline_csv):
            csv_file = centerline_csv
        else:
            self.get_logger().error(f"Could not find raceline or centerline CSV for {self.map_name}")
            return

        waypoints = []
        try:
            with open(csv_file, 'r') as f:
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
                    
                    if not self.initial_pose:
                        self.initial_pose = (wp.x_m, wp.y_m, wp.psi_rad)
        except Exception as e:
            self.get_logger().error(f"Error reading CSV {csv_file}: {e}")
            return

        if self.initial_pose:
            self.get_logger().info(f"Loaded {len(waypoints)} waypoints for {self.map_name}")
            
            # Prepare WaypointArray msg
            self.waypoints_msg = WaypointArray()
            self.waypoints_msg.header.frame_id = "map"
            self.waypoints_msg.header.stamp = self.get_clock().now().to_msg()
            self.waypoints_msg.waypoints = waypoints
            
            # Prepare MarkerArray for RViz
            self.markers_msg = MarkerArray()
            
            # Create a line strip for the track
            line_marker = Marker()
            line_marker.header.frame_id = "map"
            line_marker.header.stamp = self.waypoints_msg.header.stamp
            line_marker.ns = "raceline"
            line_marker.id = 0
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.1 # Line width
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
                
            # Close the loop
            if len(waypoints) > 0:
                p = Point()
                p.x = waypoints[0].x_m
                p.y = waypoints[0].y_m
                p.z = 0.0
                line_marker.points.append(p)
                
            self.markers_msg.markers.append(line_marker)
            
            # Publish latched data immediately
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
        msg.header.frame_id = 'map'
        
        x, y, yaw = self.initial_pose
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # yaw to quaternion
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.pose_pub.publish(msg)
        self.publish_count += 1

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TrackPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
