#!/usr/bin/env python3
import os
import time
import math
import rclpy
from rclpy.node import Node
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from matplotlib import patches
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Twist, Quaternion, Pose, Vector3
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from ament_index_python.packages import get_package_share_directory


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        
        # CAR VARIABLES
        self.LOOKAHEAD = 1.5
        self.WB = 0.3302
        
        # PROGRAM VARIABLES
        self.show_animation = True
        
        # State variables
        self.xc = 0.0
        self.yc = 0.0
        self.yaw = 0.0
        self.vel = 0.0
        self.v_prev_error = 0.0
        self.freqs = 2
        
        # Declare parameters
        self.declare_parameter('waypoint_file', 'driving_style2.csv')
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('desired_velocity', 1.0)
        self.declare_parameter('show_animation', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', '/drive')
        
        # Get parameters
        waypoint_file = self.get_parameter('waypoint_file').value
        self.LOOKAHEAD = self.get_parameter('lookahead_distance').value
        self.WB = self.get_parameter('wheelbase').value
        self.desired_velocity = self.get_parameter('desired_velocity').value
        self.show_animation = self.get_parameter('show_animation').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        
        # Load waypoints
        self.waypoints = self.read_points(waypoint_file)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {waypoint_file}')
        
        # Create subscriber and publisher
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.pose_callback,
            10
        )
        
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            1
        )
        
        # Create timer for control loop
        timer_period = 1.0 / self.freqs  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info('Pure Pursuit Node initialized')

    def read_points(self, file_name):
        """
        Load waypoints from CSV file
        """
        try:
            # First try to find in package share directory
            pkg_share = get_package_share_directory('pure_pursuit')
            file_path = os.path.join(pkg_share, file_name)
            if not os.path.exists(file_path):
                # Try current directory
                file_path = file_name
            if not os.path.exists(file_path):
                # Try parent directory
                file_path = os.path.join('..', file_name)
            
            with open(file_path) as f:
                path_points = np.loadtxt(file_path, delimiter=',')
            return path_points
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return np.array([[0, 0]])

    def pose_callback(self, data):
        """
        Get current state of the vehicle
        """
        self.xc = data.pose.pose.position.x
        self.yc = data.pose.pose.position.y
        
        # Convert Quaternions to Eulers
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        self.vel = la.norm(np.array([
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z
        ]), 2)

    def find_distance(self, x1, y1):
        distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
        return distance

    def find_distance_index_based(self, idx):
        if idx >= len(self.waypoints):
            idx = len(self.waypoints) - 1
        x1 = float(self.waypoints[idx][0])
        y1 = float(self.waypoints[idx][1])
        distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
        return distance

    def find_nearest_waypoint(self):
        """
        Get closest idx to the vehicle
        """
        curr_xy = np.array([self.xc, self.yc])
        waypoints_xy = self.waypoints[:, :2]
        nearest_idx = np.argmin(np.sum((curr_xy - waypoints_xy)**2, axis=1))
        return nearest_idx - 1

    def idx_close_to_lookahead(self, idx):
        """
        Get closest index to lookahead that is greater than the lookahead
        Wraps around to the beginning of waypoints when reaching the end
        """
        max_iterations = len(self.waypoints)  # Prevent infinite loop
        iterations = 5
        while self.find_distance_index_based(idx) < self.LOOKAHEAD:
            idx += 1
            if idx >= len(self.waypoints):
                idx = 0  # Wrap around to the beginning
            iterations += 1
            if iterations >= max_iterations:
                break  # Safety check to prevent infinite loop
        return idx if idx > 0 else len(self.waypoints) - 1

    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                     fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)
            patches.Rectangle((self.xc, self.yc), 0.35, 0.2)

    def control_loop(self):
        """
        Main control loop - runs at specified frequency
        """
        if len(self.waypoints) < 2:
            return
            
        cx = self.waypoints[:, 0]
        cy = self.waypoints[:, 1]

        nearest_idx = self.find_nearest_waypoint()
        idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
        target_x = float(self.waypoints[idx_near_lookahead][0])
        target_y = float(self.waypoints[idx_near_lookahead][1])
        
        # Velocity PID controller
        kp = 1.0
        kd = 0
        ki = 0
        dt = 1.0 / self.freqs
        v_desired = self.desired_velocity
        v_error = v_desired - self.vel
        
        # PID controller for velocity
        P_vel = kp * v_error
        I_vel = ki * v_error * dt
        D_vel = kd * (v_error - self.v_prev_error) / dt
        velocity = P_vel + I_vel + D_vel
        self.v_prev_error = v_error

        """
        PURE PURSUIT CONTROLLER
        """
        # calculate alpha (angle between the goal point and the path point)
        x_delta = target_x - self.xc
        y_delta = target_y - self.yc
        alpha = np.arctan2(y_delta, x_delta) - self.yaw

        # Set the lookahead distance depending on the speed
        lookahead = self.find_distance(target_x, target_y)
        steering_angle = np.arctan2((2 * self.WB * np.sin(alpha)), lookahead)
        
        # Set max wheel turning angle (in radians, ~30 degrees = 0.524 rad)
        max_steering = np.deg2rad(20.0)
        if steering_angle > max_steering:
            steering_angle = max_steering
        elif steering_angle < -max_steering:
            steering_angle = -max_steering
            
        # Publish AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "base_link"
        ackermann_msg.drive.speed = self.desired_velocity
        ackermann_msg.drive.steering_angle = steering_angle
        self.ackermann_pub.publish(ackermann_msg)
        
        self.get_logger().info(
            f'Steering Angle (rad): {ackermann_msg.drive.steering_angle:.3f}, '
            f'Velocity (m/s): {ackermann_msg.drive.speed:.3f}'
        )
        
        # Plot map progression
        if self.show_animation:
            plt.cla()
            # For stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )
            self.plot_arrow(float(self.xc), float(self.yc), float(self.yaw))
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(self.xc, self.yc, "-b", label="trajectory")
            plt.plot(target_x, target_y, "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Pure Pursuit Control")
            plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    
    print("RUNNING PURE-PURSUIT CODE.... \n\n")
    time.sleep(2)
    
    node = PurePursuitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
