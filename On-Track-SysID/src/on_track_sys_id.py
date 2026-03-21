#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np
import os
import yaml
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64MultiArray, String
from datetime import datetime
import sys # Ensure sys is imported
from tqdm import tqdm

from ament_index_python.packages import get_package_share_directory

# Import helpers - handle both installed and development paths
try:
    from helpers.train_model import nn_train, get_model_param
    from helpers.pacejka_formula import pacejka_formula
    from helpers.benchmarking_metrics import OnlineBenchmark
except ImportError:
    import sys
    # Add the src directory to path for development
    src_path = os.path.dirname(os.path.abspath(__file__))
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
    from helpers.train_model import nn_train, get_model_param
    from helpers.pacejka_formula import pacejka_formula
    from helpers.benchmarking_metrics import OnlineBenchmark


class OnTrackSysId(Node):
    def __init__(self):
        super().__init__('on_track_sys_id')
        # Declare parameters
        self.declare_parameter('racecar_version', 'SIM')
        self.declare_parameter('save_LUT_name', 'NUCx_on_track_pacejka')
        self.declare_parameter('plot_model', False)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ackermann_cmd_topic', '/drive')
        self.declare_parameter('benchmarking_log_interval', 100)
        # Get parameters
        self.racecar_version = self.get_parameter('racecar_version').value
        self.save_LUT_name = self.get_parameter('save_LUT_name').value
        self.plot_model = self.get_parameter('plot_model').value
        odom_topic = self.get_parameter('odom_topic').value
        ackermann_topic = self.get_parameter('ackermann_cmd_topic').value
        self.bench_log_interval = self.get_parameter('benchmarking_log_interval').value
        # Print parameters
        self.get_logger().info(f"Racecar_version: {self.racecar_version}")
        self.get_logger().info(f"Save_LUT_name: {self.save_LUT_name}")
        self.get_logger().info(f"Plot_model: {self.plot_model}")
        self.get_logger().info(f"Odom_topic: {odom_topic}")
        self.get_logger().info(f"Ackermann_topic: {ackermann_topic}")
        # Initialize variables
        self.rate = 50
        self.package_path = get_package_share_directory('on_track_sys_id')
        # Load parameters
        self.load_parameters()
        self.setup_data_storage()

        # Subscribe to the topics using the loaded parameter values
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_cb,
            1
        )
        self.ackermann_sub = self.create_subscription(
            AckermannDriveStamped,
            ackermann_topic,
            self.ackermann_cb,
            1
        )

        # Publishers for estimated state, sensor state, and error
        self.est_state_pub = self.create_publisher(Float64MultiArray, '/estimated_state', 1)
        self.sensor_state_pub = self.create_publisher(Float64MultiArray, '/sensor_state', 1)
        self.error_pub = self.create_publisher(Float64MultiArray, '/estimation_error', 1)

        # Academic benchmarking publishers
        self.bench_vy_pub = self.create_publisher(
            Float64MultiArray, '/benchmarking/vy_metrics', 1)
        self.bench_omega_pub = self.create_publisher(
            Float64MultiArray, '/benchmarking/omega_metrics', 1)
        self.bench_summary_pub = self.create_publisher(
            String, '/benchmarking/summary', 1)

        # Training completion signal (latched / transient_local)
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.training_complete_pub = self.create_publisher(
            String, '/sysid/training_complete', qos_latched)

        # Online benchmarking accumulators
        self.bench_vy = OnlineBenchmark(name='v_y (lateral velocity)')
        self.bench_omega = OnlineBenchmark(name='omega (yaw rate)')
        self.bench_sample_count = 0

        # Load model parameters for estimation
        try:
            self.model_params = get_model_param(self.racecar_version)
            self.init_model_constants()
        except Exception as e:
            self.get_logger().warn(f"Could not load model parameters: {e}")
            self.model_params = None

        self.prev_v_y = 0.0
        self.prev_omega = 0.0
        self.last_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        # Create timer for main loop
        timer_period = 1.0 / self.rate
        self.loop_timer = self.create_timer(timer_period, self.loop_callback)
        
        # State tracking
        self.data_collection_complete = False
        self.training_complete = False
        self.pbar = None

    def init_model_constants(self):
        mp = self.model_params
        self.m = mp['m']
        self.I_z = mp['I_z']
        self.l_f = mp['l_f']
        self.l_r = mp['l_r']
        self.l_wb = mp['l_wb']
        self.F_zf = self.m * 9.81 * self.l_r / self.l_wb
        self.F_zr = self.m * 9.81 * self.l_f / self.l_wb
        self.C_Pf_model = mp['C_Pf_model']
        self.C_Pr_model = mp['C_Pr_model']

    def setup_data_storage(self):
        """
        Set up storage for collected data.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        """
        Load neural network parameters from params/nn_params.yaml
        """
        yaml_file = os.path.join(self.package_path, 'params', 'nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)
    
    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.
        """
        # Auto-export without prompting in ROS2
        self.get_logger().info("Exporting data as CSV...")
        data_dir = os.path.join(self.package_path, 'data', self.racecar_version)
        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(data_dir, f'{self.racecar_version}_sys_id_data_{timestamp}.csv')
        
        # Write data to CSV file    
        with open(csv_file, mode='w') as file:
            writer = csv.writer(file)
            writer.writerow(['v_x', 'v_y', 'omega', 'delta'])
            for row in self.data:
                writer.writerow(row)
        self.get_logger().info(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z
        self.current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle
        
    def collect_data(self):
        """
        Collects data during simulation.
        """
        if self.current_state[0] > 1:  # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0)
            self.data[-1] = self.current_state
            self.counter += 1
            
            # Log progress bar every 2% to avoid spamming too much but keeping it fluid
            update_interval = max(1, self.timesteps // 50)
            if self.counter % update_interval == 0 or self.counter == self.timesteps:
                percent = (self.counter / self.timesteps) * 100
                bar_length = 20
                filled_length = int(bar_length * self.counter // self.timesteps)
                bar = '=' * filled_length + '-' * (bar_length - filled_length)
                self.get_logger().info(f"Collecting data: [{bar}] {percent:.1f}% ({self.counter}/{self.timesteps})")
                
        else:
            # Show waiting message occasionally
            if self.counter == 0 and not hasattr(self, '_waiting_logged'):
                self.get_logger().info("Waiting for car to move (velocity > 1 m/s)...")
                self._waiting_logged = True
                
        if self.counter >= self.timesteps:
            self.get_logger().info("Data collection completed.")
            return True
        return False
            
    def run_nn_train(self):
        """
        Initiates training of the neural network using collected data.
        """
        self.get_logger().info("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)
        
    def publish_estimates(self):
        """
        Calculates and publishes estimated state and error using the identified parameters.
        """
        if self.model_params is None:
            return

        # Check if we have new data based on timestamp
        if self.current_time <= self.last_time:
            return

        # Calculate dt in seconds
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        
        # If dt is too small or negative, skip
        if dt <= 0.00001:
            return
            
        # If dt is too large, reset
        if dt > 0.2:
            self.last_time = self.current_time
            self.prev_v_y = self.current_state[1]
            self.prev_omega = self.current_state[2]
            return

        self.last_time = self.current_time

        v_x = self.current_state[0]
        v_y_real = self.current_state[1]
        omega_real = self.current_state[2]
        delta = self.current_state[3]

        # Skip if car is stopped
        if v_x < 0.1:
            self.prev_v_y = v_y_real
            self.prev_omega = omega_real
            return

        # One-Step Prediction using REAL previous measurements
        alpha_f = -np.arctan((self.prev_v_y + self.prev_omega * self.l_f) / v_x) + delta
        alpha_r = -np.arctan((self.prev_v_y - self.prev_omega * self.l_r) / v_x)

        # Pacejka forces
        F_f = pacejka_formula(self.C_Pf_model, alpha_f, self.F_zf)
        F_r = pacejka_formula(self.C_Pr_model, alpha_r, self.F_zr)

        # Dynamics derivatives
        v_y_dot = (1/self.m) * (F_r + F_f * np.cos(delta) - self.m * v_x * self.prev_omega)
        omega_dot = (1/self.I_z) * (F_f * self.l_f * np.cos(delta) - F_r * self.l_r)

        # Predicted next state
        v_y_pred = self.prev_v_y + v_y_dot * dt
        omega_pred = self.prev_omega + omega_dot * dt

        # Store current real state for next prediction
        self.prev_v_y = v_y_real
        self.prev_omega = omega_real

        # Publish Sensor State
        sensor_msg = Float64MultiArray()
        sensor_msg.data = [v_x, v_y_real, omega_real, delta]
        self.sensor_state_pub.publish(sensor_msg)

        # Publish Estimated/Predicted State
        est_msg = Float64MultiArray()
        est_msg.data = [v_x, v_y_pred, omega_pred]
        self.est_state_pub.publish(est_msg)

        # Publish Error (backward-compatible)
        err_msg = Float64MultiArray()
        err_msg.data = [abs(v_y_real - v_y_pred), abs(omega_real - omega_pred)]
        self.error_pub.publish(err_msg)

        # --- Academic Benchmarking ---
        self.bench_vy.update(v_y_real, v_y_pred)
        self.bench_omega.update(omega_real, omega_pred)
        self.bench_sample_count += 1

        # Publish metric arrays [RMSE, MAE, NRMSE, MaxAE, Bias, StdDev, R²]
        vy_metrics_msg = Float64MultiArray()
        vy_metrics_msg.data = self.bench_vy.get_metrics_array()
        self.bench_vy_pub.publish(vy_metrics_msg)

        omega_metrics_msg = Float64MultiArray()
        omega_metrics_msg.data = self.bench_omega.get_metrics_array()
        self.bench_omega_pub.publish(omega_metrics_msg)

        # Publish human-readable summary
        summary_msg = String()
        summary_msg.data = (
            self.bench_vy.get_summary_string() + '\n' +
            self.bench_omega.get_summary_string()
        )
        self.bench_summary_pub.publish(summary_msg)

        # Periodic logging to terminal
        if self.bench_sample_count % self.bench_log_interval == 0:
            self.get_logger().info(
                '\n' + self.bench_vy.get_summary_string() +
                '\n' + self.bench_omega.get_summary_string()
            )

    def loop_callback(self):
        """
        Main loop callback - handles data collection, training, and estimation phases.
        """
        if not self.data_collection_complete:
            # Data Collection Phase
            if self.collect_data():
                self.data_collection_complete = True
                
        elif not self.training_complete:
            # Training Phase
            self.run_nn_train()
            self.export_data_as_csv()
            
            # Reload Parameters Phase
            self.get_logger().info("Reloading parameters for estimation...")
            try:
                self.model_params = get_model_param(self.racecar_version)
                self.init_model_constants()
                self.get_logger().info("Parameters reloaded successfully.")
            except Exception as e:
                self.get_logger().error(f"Failed to reload parameters: {e}")
            
            self.training_complete = True
            self.last_time = self.current_time

            # Publish training completion signal with identified params
            tc_msg = String()
            tc_msg.data = yaml.dump({
                'C_Pf': [float(v) for v in self.C_Pf_model],
                'C_Pr': [float(v) for v in self.C_Pr_model],
                'racecar_version': self.racecar_version,
            })
            self.training_complete_pub.publish(tc_msg)
            self.get_logger().info("Published training completion signal.")
            self.get_logger().info("Starting estimation loop...")
            
        else:
            # Estimation Phase
            self.publish_estimates()


def main(args=None):
    rclpy.init(args=args)
    
    node = OnTrackSysId()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()