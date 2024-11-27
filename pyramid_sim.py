import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
R_base = 0.8                    # Base radius for the third layer
T = 10                          # Period of rotation [s]
omega = 2 * np.pi / T           # Angular velocity
phases = [0, 2 * np.pi / 3, 4 * np.pi / 3]  # Phases for the rotating points
total_time = 2 * T                          # Total simulation time

# Function to compute the positions of the pyramid points
def compute_positions(t):
    top_point = [0, 0, 1.5]

    R2 = R_base * 0.5  # Shrinking radius for second layer
    layer2 = [
        [R2 * np.cos(omega * t + phase), R2 * np.sin(omega * t + phase), 1.0]
        for phase in phases
    ]
    R3 = R_base  # Larger radius for the third layer
    layer3 = [
        [R3 * np.cos(omega * t + phase), R3 * np.sin(omega * t + phase), 0.5]
        for phase in phases
    ]
    return [top_point] + layer2 + layer3

'Simulation'
timestep = 0.1
times = np.arange(0, total_time, timestep)
positions_over_time = [compute_positions(t) for t in times]

'Real-time 3D Plot'
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for step, t in enumerate(times):
    ax.cla()
    positions = np.array(positions_over_time[step])
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    ax.scatter(x, y, z, c='b', label='Pyramid Points')

    for i in range(3):
        ax.plot([x[0], x[i+1]], [y[0], y[i+1]], [z[0], z[i+1]], c='g') 
        ax.plot([x[i+1], x[i+4]], [y[i+1], y[i+4]], [z[i+1], z[i+4]], c='r') 

    for i in range(3):
        ax.plot([x[i+1], x[(i+1)%3+1]], [y[i+1], y[(i+1)%3+1]], [z[i+1], z[(i+1)%3+1]], c='g')
        ax.plot([x[i+4], x[(i+1)%3+4]], [y[i+4], y[(i+1)%3+4]], [z[i+4], z[(i+1)%3+4]], c='r')
  
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 2])
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("3D Pyramid Rotation Simulation")
    
    plt.pause(timestep)

plt.show()
