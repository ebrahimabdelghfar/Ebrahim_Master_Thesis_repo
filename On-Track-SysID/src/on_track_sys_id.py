#!/usr/bin/env python3

import rospy
import rospkg
import numpy as np
import os
import yaml
import csv
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from helpers.train_model import nn_train, get_model_param
from helpers.pacejka_formula import pacejka_formula
from std_msgs.msg import Float64MultiArray
from datetime import datetime
from tqdm import tqdm

class On_Track_Sys_Id:
    def __init__(self):

        rospy.init_node('on_track_sys_id', anonymous=True)

        if rospy.has_param('/racecar_version'):
            self.racecar_version = rospy.get_param('/racecar_version')
        else:
            self.racecar_version = rospy.get_param('~racecar_version')

        print("Racecar_version: ", self.racecar_version)

        self.rate = 50
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('on_track_sys_id')

        self.load_parameters()
        self.setup_data_storage()
        self.loop_rate = rospy.Rate(self.rate)

        self.save_LUT_name = rospy.get_param('~save_LUT_name')
        self.plot_model = rospy.get_param('~plot_model')

        # Get topic names from parameters (with default values if not set)
        odom_topic = rospy.get_param('~odom_topic', '/car_state/odom')
        ackermann_topic = rospy.get_param('~ackermann_cmd_topic', '/vesc/high_level/ackermann_cmd_mux/input/nav_1')

        # Subscribe to the topics using the loaded parameter values
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb)
        rospy.Subscriber(ackermann_topic, AckermannDriveStamped, self.ackermann_cb)

        # Publishers for estimated state, sensor state, and error
        self.est_state_pub = rospy.Publisher('/estimated_state', Float64MultiArray, queue_size=1)
        self.sensor_state_pub = rospy.Publisher('/sensor_state', Float64MultiArray, queue_size=1)
        self.error_pub = rospy.Publisher('/estimation_error', Float64MultiArray, queue_size=1)

        # Load model parameters for estimation
        try:
            self.model_params = get_model_param(self.racecar_version)
            self.init_model_constants()
        except Exception as e:
            rospy.logwarn(f"Could not load model parameters: {e}")
            self.model_params = None

        self.prev_v_y = 0.0
        self.prev_omega = 0.0
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

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

        Initializes data storage and related variables based on parameters loaded from 'nn_params'.
        """
        self.data_duration = self.nn_params['data_collection_duration']
        self.timesteps = self.data_duration * self.rate
        self.data = np.zeros((self.timesteps, 4))
        self.counter = 0
        self.current_state = np.zeros(4)

    def load_parameters(self):
        """
        This function loads parameters neural network parameters from 'params/nn_params.yaml' and stores them in self.nn_params.
        """

        yaml_file = os.path.join(self.package_path, 'params/nn_params.yaml')
        with open(yaml_file, 'r') as file:
            self.nn_params = yaml.safe_load(file)
    
    def export_data_as_csv(self):
        """
        Export collected data as a CSV file.

        Prompts the user to confirm exporting data. If confirmed, data is saved as a CSV file
        including velocity components (v_x, v_y), yaw rate (omega), and steering angle (delta).
        """
        # Prompt user for confirmation to export data
        user_input = input("\033[33m[WARN] Press 'Y' and then ENTER to export data as CSV, or press ENTER to continue without dumping: \033[0m")
        if user_input.lower() == 'y':
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
                    writer.writerow(row)  # Each row contains v_x, v_y, omega, delta
            rospy.loginfo(f"DATA HAS BEEN EXPORTED TO: {csv_file}")

    def odom_cb(self, msg):
        self.current_state[0] = msg.twist.twist.linear.x
        self.current_state[1] = msg.twist.twist.linear.y
        self.current_state[2] = msg.twist.twist.angular.z
        self.current_time = msg.header.stamp
        
    def ackermann_cb(self, msg):
        self.current_state[3] = msg.drive.steering_angle
        
    def collect_data(self):
        """
        Collects data during simulation.

        Adds the current state to the data array and updates the counter.
        Closes the progress bar and logs a message if data collection is complete.
        """
        if self.current_state[0] > 1: # Only collect data when the car is moving
            self.data = np.roll(self.data, -1, axis=0) # Shift data up by one row
            self.data[-1] = self.current_state
            self.counter += 1
            self.pbar.update(1)
        if self.counter == self.timesteps + 1:
            self.pbar.close()
            rospy.loginfo("Data collection completed.")
            
    def run_nn_train(self):
        """
        Initiates training of the neural network using collected data.
        """
        rospy.loginfo("Training neural network...")
        nn_train(self.data, self.racecar_version, self.save_LUT_name, self.plot_model)
        
    def publish_estimates(self):
        """
        Calculates and publishes estimated state and error using the identified parameters.
        Uses one-step prediction: computes predicted next state from REAL measurements,
        then compares with REAL measurements at the next timestep.
        """
        if self.model_params is None:
            return

        # Check if we have new data based on timestamp
        if self.current_time <= self.last_time:
            return

        dt = (self.current_time - self.last_time).to_sec()
        
        # If dt is too small (duplicate) or negative, skip
        if dt <= 0.00001:
            return
            
        # If dt is too large (e.g. first run or pause), reset/skip integration to avoid jumps
        if dt > 0.2:
            self.last_time = self.current_time
            # Store current real state for next prediction
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
        # Use previous real state for slip angle calculation (not simulated state)
        alpha_f = -np.arctan((self.prev_v_y + self.prev_omega * self.l_f) / v_x) + delta
        alpha_r = -np.arctan((self.prev_v_y - self.prev_omega * self.l_r) / v_x)

        # Pacejka forces
        F_f = pacejka_formula(self.C_Pf_model, alpha_f, self.F_zf)
        F_r = pacejka_formula(self.C_Pr_model, alpha_r, self.F_zr)

        # Dynamics derivatives
        v_y_dot = (1/self.m) * (F_r + F_f * np.cos(delta) - self.m * v_x * self.prev_omega)
        omega_dot = (1/self.I_z) * (F_f * self.l_f * np.cos(delta) - F_r * self.l_r)

        # Predicted next state (one-step prediction from previous real state)
        v_y_pred = self.prev_v_y + v_y_dot * dt
        omega_pred = self.prev_omega + omega_dot * dt

        # Store current real state for next prediction
        self.prev_v_y = v_y_real
        self.prev_omega = omega_real

        # Publish Sensor State [v_x, v_y_real, omega_real, delta]
        sensor_msg = Float64MultiArray()
        sensor_msg.data = [v_x, v_y_real, omega_real, delta]
        self.sensor_state_pub.publish(sensor_msg)

        # Publish Estimated/Predicted State [v_x, v_y_pred, omega_pred]
        est_msg = Float64MultiArray()
        est_msg.data = [v_x, v_y_pred, omega_pred]
        self.est_state_pub.publish(est_msg)

        # Publish Error [v_y_error, omega_error] (real - predicted)
        err_msg = Float64MultiArray()
        err_msg.data = [abs(v_y_real - v_y_pred), abs(omega_real - omega_pred)]
        self.error_pub.publish(err_msg)

    def loop(self):
        """
        Main loop for data collection, training, and exporting.

        This loop continuously collects data until completion, then runs neural network training
        and exports the collected data as CSV. After training, it reloads the parameters and 
        starts publishing estimates.
        """
        self.pbar = tqdm(total=self.timesteps, desc='Collecting data', ascii=True)
        
        # Data Collection Phase
        while not rospy.is_shutdown():
            self.collect_data()
            if self.counter == self.timesteps + 1:
                break
            self.loop_rate.sleep()
            
        if rospy.is_shutdown():
            return

        # Training Phase
        self.run_nn_train()
        self.export_data_as_csv()
        
        # Reload Parameters Phase
        rospy.loginfo("Reloading parameters for estimation...")
        try:
            self.model_params = get_model_param(self.racecar_version)
            self.init_model_constants()
            rospy.loginfo("Parameters reloaded successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to reload parameters: {e}")
        
        # Estimation Phase
        rospy.loginfo("Starting estimation loop...")
        self.last_time = self.current_time
        while not rospy.is_shutdown():
            self.publish_estimates()
            self.loop_rate.sleep()
if __name__ == '__main__':
    sys_id = On_Track_Sys_Id()
    sys_id.loop()