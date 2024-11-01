import matplotlib.pyplot as plt
import numpy as np

# Parameters
epsilon = 0.5  # Neighborhood distance in x-y plane
z_epsilon = 0.5  # Allowed variation in z-axis

# Scaled waypoints for plotting
waypoints = [(3 * x, 3 * y, z) for x, y, z 
in [
    (0.5, 0.0, 1.0), 
    (0.5, 0.5, 1.0), 
    (0.0, 0.5, 1.0), 
    (0.0, 0.0, 1.0), 
    (0.5, 0.0, 1.0)  # Close the square
]]

# Plot the path and neighborhoods
x, y, z = zip(*waypoints)
plt.plot(x, y, marker='o')
plt.axis([-2, 4, -2, 4])  # 4m by 4m axis
plt.xlabel('x position (meters)')
plt.ylabel('y position (meters)')
plt.title(f'Crazyflie Square Trajectory with Neighborhoods (epsilon = {epsilon}, z leeway = {z_epsilon})')
plt.grid(True)

# Draw epsilon neighborhoods (circles around each waypoint in x-y plane)
for wx, wy, _ in waypoints:
    neighborhood = plt.Circle((wx, wy), epsilon, color='gray', alpha=0.3)
    plt.gca().add_patch(neighborhood)

plt.show()
