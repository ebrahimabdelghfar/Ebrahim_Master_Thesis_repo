#!/usr/bin/env python3
import os
import time
import math
import rospy
import rospkg
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from matplotlib import patches
from rospkg import RosPack
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Twist, Quaternion, Pose, Point, Vector3
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# GLOBAL VARIABLES 
xc = 0.0
yc = 0.0
yaw = 0.0
vel = 0.0
idx = 0
waypoints = []
v_prev_error = 0.0
freqs = 2

# CAR VARIABLES
# LOOKAHEAD = 1.5 # 1.5
# WB = 0.3421
LOOKAHEAD = 3.5 # 1.5
WB = 2.1

# PROGRAM VARIABLES
pure_pursuit_flag = True
show_animation = True

def read_points():
   """
   CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
   """
   file_name = 'wp_file.csv' #'racecar_walker.csv'
   file_path = file_name
   with open(file_path) as f:
      path_points = np.loadtxt(file_path, delimiter = ',')
   return path_points

def pose_callback(data):
	"""
	Get current state of the vehicle
	"""
	global xc, yc, yaw, vel
	xc = data.pose.pose.position.x
	yc = data.pose.pose.position.y
	
	# Convert Quaternions to Eulers
	qx = data.pose.pose.orientation.x
	qy = data.pose.pose.orientation.y
	qz = data.pose.pose.orientation.z
	qw = data.pose.pose.orientation.w
	quaternion = (qx,qy,qz,qw)
	euler = euler_from_quaternion(quaternion)
	yaw = euler[2]
	vel = la.norm(np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]),2)

def find_distance(x1, y1):
	global xc, yc, yaw, waypoints
	distance = math.sqrt((x1 - xc) ** 2 + (y1 - yc) ** 2)
	return distance

def find_distance_index_based(idx):
	global xc, yc, yaw, waypoints
	if idx >= len(waypoints):
		idx = len(waypoints) - 1
	x1 = float(waypoints[idx][0])
	y1 = float(waypoints[idx][1])
	distance = math.sqrt((x1 - xc) ** 2 + (y1 - yc) ** 2)
	return distance

def find_nearest_waypoint():
	"""
	Get closest idx to the vehicle
	"""
	global xc, yc, yaw, waypoints, WB
	curr_xy = np.array([xc, yc])
	waypoints_xy = waypoints[:, :2]
	nearest_idx = np.argmin(np.sum((curr_xy - waypoints_xy)**2, axis=1))
	return nearest_idx - 1

def idx_close_to_lookahead(idx):
	"""
	Get closest index to lookahead that is greater than the lookahead
	"""
	global LOOKAHEAD
	while find_distance_index_based(idx) < LOOKAHEAD:
		idx += 1 
		if idx >= len(waypoints):
			break
	return idx - 1 

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
	"""
	Plot arrow
	"""
	if not isinstance(x, float):
		for ix, iy, iyaw in zip(x, y, yaw):
			plot_arrow(ix, iy, iyaw)
	else:
		plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw), fc=fc, ec=ec, head_width=width, head_length=width)
		plt.plot(x, y)
		patches.Rectangle((xc,yc), 0.35,0.2)

def pure_pursuit():
	global xc, yc, yaw, vel, waypoints, v_prev_error, freqs, idx,LOOKAHEAD, WB, pure_pursuit_flag, show_animation

	# Initialize the message, subscriber and publisher
	ackermann_msg = AckermannDriveStamped()
	rospy.Subscriber("/aft_mapped_adjusted", Odometry, pose_callback) 
	#define control_topics
	ackermann_pub = rospy.Publisher("/ackermann_cmd", AckermannDriveStamped, queue_size=1)

	cx = waypoints[:, 0]; cy = waypoints[:, 1]

	while not rospy.is_shutdown():
			nearest_idx = find_nearest_waypoint()
			idx_near_lookahead = idx_close_to_lookahead(nearest_idx) 
			target_x = float(waypoints[idx_near_lookahead][0])
			target_y = float(waypoints[idx_near_lookahead][1])
			# Velocity PID controller
			kp = 1.0 # 1.0
			kd = 0
			ki = 0
			# PID controller for velocity
			dt = 1.0 / freqs
			v_desired = 4.0
			v_error = v_desired - vel
			# PID controller for velocity
			P_vel = kp * v_error
			I_vel = ki * v_error * dt
			D_vel = kd * (v_error - v_prev_error) / dt
			# PID controller for velocity
			velocity = P_vel + I_vel + D_vel
			v_prev_error = v_error

			"""
			PURE PURSUIT CONTROLLER
			"""

			# calculate alpha (angle between the goal point and the path point)
			x_delta = target_x - xc
			y_delta = target_y - yc
			alpha = np.arctan2(y_delta , x_delta) - yaw

			# front of the vehicle is 0 degrees right +90 and left -90 hence we need to convert our alpha
			if alpha > np.pi / 2:
				alpha -= np.pi
			if alpha < -np.pi / 2:
				alpha += np.pi

			# Set the lookahead distance depending on the speed
			lookahead = find_distance(target_x, target_y)
			steering_angle = np.arctan2((2 * WB * np.sin(alpha)) , lookahead)
			# Set max wheel turning angle (in radians, ~30 degrees = 0.524 rad)
			max_steering = np.deg2rad(30.0)
			if steering_angle > max_steering:
				steering_angle = max_steering
			elif steering_angle < -max_steering:
				steering_angle = -max_steering
			# Publish AckermannDriveStamped message
			ackermann_msg.header.stamp = rospy.Time.now()
			ackermann_msg.header.frame_id = "base_link"
			ackermann_msg.drive.speed = velocity  # velocity in m/s
			ackermann_msg.drive.steering_angle = -steering_angle  # steering angle in radians
			ackermann_pub.publish(ackermann_msg)
			print("Steering Angle (rad): ", ackermann_msg.drive.steering_angle, "Velocity (m/s): ", ackermann_msg.drive.speed)
			# Plot map progression
			if show_animation:
				plt.cla()
				# For stopping simulation with the esc key.
				plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
				plot_arrow(float(xc), float(yc), float(yaw))
				plt.plot(cx, cy, "-r", label = "course")
				plt.plot(xc, yc, "-b", label = "trajectory")
				plt.plot(target_x, target_y, "xg", label = "target")
				plt.axis("equal")
				plt.grid(True)
				plt.title("Pure Pursuit Control" + str(1))
				plt.pause(0.001)


if __name__=='__main__':
   rospy.init_node('pure_pursuit')
   r = rospy.Rate(freqs)
   print("RUNNING PURE-PURSUIT CODE.... \n\n")
   time.sleep(2)
   waypoints = read_points()
   pure_pursuit()
