import numpy as np
import matplotlib.pyplot as plt  # Importing the plotting library
from mpl_toolkits.mplot3d import Axes3D

# Define the flight area dimensions
width = 5  # 5 meters along the x-axis
height = 8  # 8 meters along the y-axis

# Define the waypoint spacing and edge offset
spacing = 2.0  # meters between waypoints
edge_offset = 0.5  # meters from the edge
# edge_offset = spacing/2 (assuming square ground projection of camera FOV)

# Adjust the grid to have the origin in the center
x_points = np.arange(-width/2 + edge_offset, width/2 - edge_offset + spacing, spacing)
y_points = np.arange(-height/2 + edge_offset, height/2 - edge_offset + spacing, spacing)
print(f"x positions {x_points}")
print(f"y positions {y_points}")

# Generate waypoints
waypoints = []

# Traverse the area in a snake-like pattern
for i, x in enumerate(x_points):
    if i % 2 == 0:  # left to right
        for y in y_points:
            waypoints.append((x, y, 0.0))  # Initialize z-axis at 0.0
    else:  # right to left
        for y in reversed(y_points):
            waypoints.append((x, y, 0.0))  # Initialize z-axis at 0.0


# Define takeoff and landing waypoints (indexes in the waypoints list)
takeoff_index = 0  # Takeoff at the nth waypoint
landing_index = len(waypoints)  # Land at the last waypoint

# Set the z-coordinates for the takeoff, in-flight, and landing altitudes
for i in range(takeoff_index, landing_index-1):
    waypoints[i] = (waypoints[i][0], waypoints[i][1], 1.25)  # Set altitude to 1.25m

# Print the generated waypoints
for waypoint in waypoints:
    print(waypoint)

# Print the number of waypoints
print(f"Number of waypoints: {len(waypoints)}")

# Plot the waypoints
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, z coordinates from waypoints
x_coords = [wp[0] for wp in waypoints]
y_coords = [wp[1] for wp in waypoints]
z_coords = [wp[2] for wp in waypoints]

# Plot waypoints
ax.plot(x_coords, y_coords, z_coords, marker='o')

# Label the axes
ax.set_xlabel('X-axis (meters)')
ax.set_ylabel('Y-axis (meters)')
ax.set_zlabel('Z-axis (meters)')

# Set plot limits for better visualization
ax.set_xlim([-width/2, width/2])
ax.set_ylim([-height/2, height/2])
ax.set_zlim([0, 1.5])

# Show the plot
plt.show()

#djbvkisadnb
