import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Set altitude & YAW
alt = 1.3
yaw = 0.0

waypoints = []
# Add takeoff waypoint (over origin of map)
waypoints.append([0, 0, alt, 0])

# NOTE: in demo y is lateral, x is forward (facing demo area)
width = 8
height = 5
edge_offset = 1
spacing = 1.5

# Adjust the grid to have the origin in the center and stay within bounds
x_points = np.arange(-width/2 + edge_offset, width/2 - edge_offset + spacing/2, spacing)
y_points = np.arange(-height/2 + edge_offset, height/2 - edge_offset + spacing/2, spacing)

# Traverse the area in a straight line along the y-axis (long axis)
for y in y_points:
    for x in x_points:
        waypoints.append([x, y, alt, yaw])  # Initialize with z=0.0 and yaw=0.0
    x_points = x_points[::-1]  # Reverse x_points to keep a straight line in y direction

# Add pre-landing waypoint (origin of map)
waypoints.append([0, 0, alt, 0])

#print(f"Waypoint List Generated: {waypoints}")
print(f"x positions: {x_points}")
print(f"y positions: {y_points}")
print(f" waypoints: {waypoints}")

# Plot the waypoints
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, z coordinates from waypoints
x_coords = [wp[0] for wp in waypoints]
y_coords = [wp[1] for wp in waypoints]
z_coords = [wp[2] for wp in waypoints]

# Plot waypoints
ax.plot(x_coords, y_coords, z_coords, marker='o', linestyle='-', label='Waypoints')

# Plot the lateral and longitudinal boundaries
ax.plot([-width/2, width/2, width/2, -width/2, -width/2], 
        [-height/2, -height/2, height/2, height/2, -height/2], 
        [alt, alt, alt, alt, alt], color='r', linestyle='--', label='Boundaries')

# Label the axes
ax.set_xlabel('X-axis (meters)')
ax.set_ylabel('Y-axis (meters)')
ax.set_zlabel('Z-axis (meters)')

# Set plot limits for better visualization
ax.set_xlim([-width/2, width/2])
ax.set_ylim([-height/2, height/2])
ax.set_zlim([0, 1.5])

# Add a legend
ax.legend()

# Show the plot
plt.show()
