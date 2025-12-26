#!/usr/bin/env python3

'''
Displays the map of waypoints
'''

# Imports
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
import os

# GLOBAL VARIABLES 
xc = 0
yc = 0
yaw = 0 
idx = 0
waypoints = []

def read_points():
    # CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
    file_name = 'IPG_waypoint.csv'
    file_path = file_name
    with open(file_path) as f:
        path_points = np.loadtxt(file_path, delimiter=',')
    return path_points

if __name__=='__main__':
    waypoints = read_points()
    plt.cla()
    # PURE PURSUIT CODE 
    cx = []
    cy = []
    for point in waypoints:
        cx.append(float(point[0]))
        cy.append(float(point[1]))
    plt.plot(cx, cy, "-r", label="fine")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Pure Pursuit Control" + str(1))
    plt.show()
