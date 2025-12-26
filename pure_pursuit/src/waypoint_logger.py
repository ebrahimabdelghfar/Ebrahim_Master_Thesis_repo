#!/usr/bin/env python3
"""
Waypoint Logger for ROS2
Records waypoints from odometry topic and saves them to a CSV file.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import atexit
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry


class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')
        
        self.file_name = 'wp_file.csv'
        self.file = open(self.file_name, 'w')
        self.ctr = 0
        self.euler = None
        
        self.get_logger().info(f"Saving to file - {self.file_name}")
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.save_waypoint,
            10
        )
        
        atexit.register(self.shutdown)
    
    def save_waypoint(self, data: Odometry):
        quaternion = np.array([
            data.pose.pose.orientation.x, 
            data.pose.pose.orientation.y, 
            data.pose.pose.orientation.z, 
            data.pose.pose.orientation.w
        ])
        self.euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([
            data.twist.twist.linear.x, 
            data.twist.twist.linear.y, 
            data.twist.twist.linear.z
        ]), 2)
        
        if self.ctr == 1:
            self.file.write('%f, %f, %f, %f\n' % (
                data.pose.pose.position.x, 
                data.pose.pose.position.y, 
                self.euler[2], 
                speed
            ))
            self.get_logger().info(f"Waypoint: {data.pose.pose.position.x:.2f}, {data.pose.pose.position.y:.2f}")
            self.ctr = 0
        else:
            self.ctr += 1

    def shutdown(self):
        self.file.close()
        print('Goodbye')


def main(args=None):
    rclpy.init(args=args)
    
    print('Saving waypoints...')
    node = WaypointLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
