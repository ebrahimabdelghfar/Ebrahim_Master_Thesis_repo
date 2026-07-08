#!/usr/bin/env python3
import os
import csv
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ament_index_python.packages import get_package_share_directory

class SpawnPublisher(Node):
    def __init__(self):
        super().__init__('spawn_publisher')
        
        # Declare parameters
        self.declare_parameter('map_name', 'YasMarina')
        self.declare_parameter('racetracks_dir', '')
        
        # Publisher for initial pose
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Create a timer to publish multiple times initially to ensure simulator gets it
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.publish_count = 0
        self.max_publishes = 5

        # Extract map_name
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        self.racetracks_dir = self.get_parameter('racetracks_dir').get_parameter_value().string_value
        
        if not self.racetracks_dir:
            # Fallback for racetracks dir assuming typical workspace structure
            # e.g., if we run from install share: install/f1tenth_simulator/share/f1tenth_simulator/scripts
            # but usually we want to find Ebrahim_Master_Thesis_repo
            # A safer fallback is passing it from launch file
            pass

        self.initial_pose = None
        self.load_initial_pose()

    def load_initial_pose(self):
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

        # Parse CSV. Format: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f, delimiter=';')
                for row in reader:
                    if not row or row[0].startswith('#'):
                        continue
                    # First data row
                    x_m = float(row[1])
                    y_m = float(row[2])
                    psi_rad = float(row[3])
                    self.initial_pose = (x_m, y_m, psi_rad)
                    break
        except Exception as e:
            self.get_logger().error(f"Error reading CSV {csv_file}: {e}")
            return

        if self.initial_pose:
            self.get_logger().info(f"Loaded initial pose for {self.map_name}: x={self.initial_pose[0]}, y={self.initial_pose[1]}, yaw={self.initial_pose[2]}")

    def publish_pose(self):
        if not self.initial_pose:
            self.timer.cancel()
            return
            
        if self.publish_count >= self.max_publishes:
            self.timer.cancel()
            self.get_logger().info("Finished publishing initial pose. Shutting down node.")
            raise SystemExit # Shutdown node cleanly

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
        node = SpawnPublisher()
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info("Done")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
