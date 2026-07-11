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
from std_msgs.msg import Header, ColorRGBA, Float64, Bool
from rcl_interfaces.msg import SetParametersResult
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Twist, Quaternion, Pose, Vector3
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from ament_index_python.packages import get_package_share_directory
from f1tenth_msgs.msg import WaypointArray
from rclpy.qos import QoSProfile, DurabilityPolicy


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
        self.freqs = 50
        
        # Declare parameters
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('kp_vel', 2.0)
        self.declare_parameter('ki_vel', 0.05)
        self.declare_parameter('kd_vel', 0.1)
        self.declare_parameter('show_animation', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('drive_topic', 'pp/drive_cmd')
        self.declare_parameter('waypoint_topic', '/raceline_waypoints')
        self.declare_parameter('standalone_mode', False)
        self.declare_parameter('enable_topic', 'Start_Working_pp')
        self.declare_parameter('state_topic', 'pp_state')
        self.declare_parameter('health_topic', 'pp_health')
        self.declare_parameter('stale_data_timeout_s', 0.5)

        # Get parameters
        self.LOOKAHEAD = self.get_parameter('lookahead_distance').value
        self.WB = self.get_parameter('wheelbase').value
        self.kp_vel = self.get_parameter('kp_vel').value
        self.ki_vel = self.get_parameter('ki_vel').value
        self.kd_vel = self.get_parameter('kd_vel').value
        self.show_animation = self.get_parameter('show_animation').value
        odom_topic = self.get_parameter('odom_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        waypoint_topic = self.get_parameter('waypoint_topic').value
        self.standalone_mode = self.get_parameter('standalone_mode').value
        enable_topic = self.get_parameter('enable_topic').value
        state_topic = self.get_parameter('state_topic').value
        health_topic = self.get_parameter('health_topic').value
        self.stale_data_timeout_s = self.get_parameter('stale_data_timeout_s').value

        self.waypoints = np.array([])
        self.integral_error = 0.0
        # True once managed startup is authorized by adaptive_controller_manager;
        # ignored entirely when standalone_mode is True.
        self.start_working = False
        self.last_odom_time = None
        
        # Create subscriber and publisher
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.pose_callback,
            1
        )
        
        # QoS for latched topic
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.waypoint_sub = self.create_subscription(
            WaypointArray,
            waypoint_topic,
            self.waypoint_callback,
            latched_qos
        )
        
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            1
        )

        self.enable_sub = self.create_subscription(
            Bool,
            enable_topic,
            self.enable_callback,
            1
        )

        self.state_pub = self.create_publisher(Bool, state_topic, 1)
        self.health_pub = self.create_publisher(Bool, health_topic, 1)

        # Create timer for control loop
        timer_period = 1.0 / self.freqs  # seconds
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        # Publishers for PID tuning graphs
        self.target_vel_pub = self.create_publisher(Float64, 'debug/target_velocity', 1)
        self.current_vel_pub = self.create_publisher(Float64, 'debug/current_velocity', 1)
        self.control_effort_pub = self.create_publisher(Float64, 'debug/control_effort', 1)
        
        # Add parameter callback for online tuning
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Pure Pursuit Node initialized')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp_vel':
                self.kp_vel = param.value
                self.get_logger().info(f'Updated kp_vel to {self.kp_vel}')
            elif param.name == 'ki_vel':
                self.ki_vel = param.value
                self.get_logger().info(f'Updated ki_vel to {self.ki_vel}')
            elif param.name == 'kd_vel':
                self.kd_vel = param.value
                self.get_logger().info(f'Updated kd_vel to {self.kd_vel}')
            elif param.name == 'lookahead_distance':
                self.LOOKAHEAD = param.value
                self.get_logger().info(f'Updated lookahead_distance to {self.LOOKAHEAD}')
        return SetParametersResult(successful=True)

    def enable_callback(self, msg):
        self.start_working = msg.data

    def waypoint_callback(self, msg):
        """
        Callback to receive raceline waypoints
        """
        if len(msg.waypoints) == 0:
            return
            
        # Convert WaypointArray into a numpy array: [x, y, vx, ax]
        waypoints_list = []
        for wp in msg.waypoints:
            waypoints_list.append([wp.x_m, wp.y_m, wp.vx_mps, wp.ax_mps2])
            
        self.waypoints = np.array(waypoints_list)
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')

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
        self.last_odom_time = self.get_clock().now()

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
        return nearest_idx

    def idx_close_to_lookahead(self, idx):
        """
        Get closest index to lookahead that is greater than the lookahead
        Wraps around to the beginning of waypoints when reaching the end
        """
        max_iterations = len(self.waypoints)  # Prevent infinite loop
        iterations = 0
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
        has_waypoints = len(self.waypoints) >= 2
        has_odom = self.last_odom_time is not None
        is_healthy = (
            has_odom and
            (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
            < self.stale_data_timeout_s
        )

        self.state_pub.publish(Bool(data=has_waypoints and has_odom))
        self.health_pub.publish(Bool(data=is_healthy))

        if not has_waypoints:
            return

        cx = self.waypoints[:, 0]
        cy = self.waypoints[:, 1]

        nearest_idx = self.find_nearest_waypoint()
        idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
        target_x = float(self.waypoints[idx_near_lookahead][0])
        target_y = float(self.waypoints[idx_near_lookahead][1])
        
        target_vx = float(self.waypoints[idx_near_lookahead][2])
        target_ax = float(self.waypoints[idx_near_lookahead][3])
        
        # Velocity PID controller
        # The simulator only respects drive.speed (not drive.acceleration).
        # It uses its own internal P controller to reach the commanded speed.
        # Therefore, PID output must be used to compute the speed setpoint.
        dt = 1.0 / self.freqs
        v_error = target_vx - self.vel
        self.integral_error += v_error * dt
        
        # Anti-windup
        if self.integral_error > 5.0:
            self.integral_error = 5.0
        elif self.integral_error < -5.0:
            self.integral_error = -5.0
            
        P_vel = self.kp_vel * v_error
        I_vel = self.ki_vel * self.integral_error
        D_vel = self.kd_vel * (v_error - self.v_prev_error) / dt
        
        # PID output acts as a direct velocity correction
        pid_output = P_vel + I_vel + D_vel
        # Commanded speed is the target velocity plus the PID correction
        commanded_speed = target_vx + pid_output
        # Clamp to non-negative speed
        commanded_speed = max(0.0, commanded_speed)
        
        self.v_prev_error = v_error

        # Publish debug values for rqt_plot
        msg_target_vel = Float64()
        msg_target_vel.data = float(target_vx)
        self.target_vel_pub.publish(msg_target_vel)
        
        msg_current_vel = Float64()
        msg_current_vel.data = float(self.vel)
        self.current_vel_pub.publish(msg_current_vel)
        
        msg_control_effort = Float64()
        msg_control_effort.data = float(pid_output)
        self.control_effort_pub.publish(msg_control_effort)

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
        
        # Set max wheel turning angle (simulator limit is 0.4189 rad ≈ 24 deg)
        max_steering = 0.4189
        if steering_angle > max_steering:
            steering_angle = max_steering
        elif steering_angle < -max_steering:
            steering_angle = -max_steering
            
        # Publish AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = "base_link"
        ackermann_msg.drive.speed = commanded_speed
        ackermann_msg.drive.steering_angle = steering_angle
        if self.standalone_mode or self.start_working:
            self.ackermann_pub.publish(ackermann_msg)

        # self.get_logger().info(
        #     f'Steer: {steering_angle:.3f} rad, '
        #     f'Cmd Speed: {commanded_speed:.2f}, '
        #     f'Target Vx: {target_vx:.2f}, '
        #     f'Current V: {self.vel:.2f}'
        # )
        
        # Plot map progression
        # if self.show_animation:
        #     plt.cla()
        #     # For stopping simulation with the esc key.
        #     plt.gcf().canvas.mpl_connect(
        #         'key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None]
        #     )
        #     self.plot_arrow(float(self.xc), float(self.yc), float(self.yaw))
        #     plt.plot(cx, cy, "-r", label="course")
        #     plt.plot(self.xc, self.yc, "-b", label="trajectory")
        #     plt.plot(target_x, target_y, "xg", label="target")
        #     plt.axis("equal")
        #     plt.grid(True)
        #     plt.title("Pure Pursuit Control")
        #     plt.pause(0.001)


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
