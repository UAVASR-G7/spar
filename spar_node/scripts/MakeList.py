import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the flight area dimensions
width = 5  # 5 meters along the x-axis
height = 8  # 8 meters along the y-axis

# Define the waypoint spacing and edge offset
edge_offset = 0.5  # Keep the edge offset the same
spacing = 2 # no complete solution of edge_offset in terms of spacing (can approximate tho)

# Adjust the grid to have the origin in the center and stay within bounds
x_points = np.arange(-width/2 + edge_offset, width/2 - edge_offset + spacing/2, spacing)
y_points = np.arange(-height/2 + edge_offset, height/2 - edge_offset + spacing/2, spacing)

print(f"x positions: {x_points}")
print(f"y positions: {y_points}")

# Generate waypoints
waypoints = []

# Traverse the area in a straight line along the y-axis (long axis)
for x in x_points:
    for y in y_points:
        waypoints.append((x, y, 0.0))  # Initialize z-axis at 0.0
    y_points = y_points[::-1]  # Reverse y_points to keep a straight line in y direction

# Define takeoff and landing waypoints (indexes in the waypoints list)
takeoff_index = 0  # Takeoff at this waypoint
landing_index = len(waypoints)  # Land at this waypoint

# Set the z-coordinates for the takeoff, in-flight, and landing altitudes
for i in range(takeoff_index, landing_index):
    waypoints[i] = (waypoints[i][0], waypoints[i][1], 1.25)  # Set altitude to 1.25m