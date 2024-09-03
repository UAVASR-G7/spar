import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Set altitude & YAW
alt = 2
yaw = 0.0

waypoints = []
# Add takeoff waypoint (over origin of map)
waypoints.append([0, 0, alt, 0])

# Given width and height (cannot change these)
width = 8
height = 5

# Define edge offsets
x_edge_offset = 0.5
y_edge_offset = 0.5

# Calculate spacings based on the defined edge offsets
available_x_space = width - 2 * x_edge_offset
available_y_space = height - 2 * y_edge_offset

# Calculate spacing to ensure symmetry
x_spacing = available_x_space / (int(available_x_space) // 2)
y_spacing = available_y_space / (int(available_y_space) // 2)

# Adjust the grid to have the origin in the center and stay within bounds
x_points = np.arange(-width/2 + x_edge_offset, width/2 - x_edge_offset + x_spacing, x_spacing)
y_points = np.arange(-height/2 + y_edge_offset, height/2 - y_edge_offset + y_spacing, y_spacing)

# Traverse the area in a straight line along the x-axis (long axis)
for x in x_points:
    for y in y_points:
        waypoints.append([x, y, alt, yaw])  # Initialize with z=0.0 and yaw=0.0
    y_points = y_points[::-1]  # Reverse y_points to keep a straight line in x direction

# Add pre-landing waypoint (origin of map)
waypoints.append([0, 0, alt, 0])

# Remove the middle waypoint if three consecutive waypoints are aligned
def remove_middle_if_aligned(waypoints):
    new_waypoints = []
    i = 0
    while i < len(waypoints):
        if i > 0 and i < len(waypoints) - 1:
            prev_wp = waypoints[i - 1]
            curr_wp = waypoints[i]
            next_wp = waypoints[i + 1]
            
            # Check if the current waypoint is in a row (aligned on either axis)
            if (prev_wp[0] == curr_wp[0] == next_wp[0]) or (prev_wp[1] == curr_wp[1] == next_wp[1]):
                # Skip the current waypoint (remove it)
                i += 1
                continue

        new_waypoints.append(waypoints[i])
        i += 1
    return new_waypoints

# Apply the function to remove middle waypoints
#waypoints = remove_middle_if_aligned(waypoints)

#print(f"Waypoint List Generated: {waypoints}")
print(f"x positions: {x_points}")
print(f"y positions: {y_points}")
print(f"altitude: {alt}")
#print(f" waypoints: {waypoints}")

# Function to plot the waypoints
def plot_waypoints(waypoints, width, height, alt):
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
    ax.set_zlim([0, alt])

    # Add a legend
    ax.legend()

    # Show the plot
    plt.show()

# Check all generated waypoints are within bounds (height, width & alt)
within_bounds = True
for wp in waypoints:
    x, y, z, _ = wp # "_" ignores YAW
    if not (-height/2 <= y <= height/2 and -width/2 <= x <= width/2 and z <= alt):
        within_bounds = False
        print(f"Waypoint {wp} is out of bounds!, all waypoints nulled")
        waypoints = 0
        break

if within_bounds:
    print("All waypoints are within bounds.")
    plot_waypoints(waypoints, width, height, alt)
