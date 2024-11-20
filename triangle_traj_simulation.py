import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
L = 1                                       # Side length of the equilateral triangle [m]
R = L / np.sqrt(3)                          # Radius of the circular motion [m]
T = 10
h=1                                      # Period of rotation [s
omega = 2 * np.pi / T                       # Angular velocity [rad/s]
phases = [0, 2 * np.pi / 3, 4 * np.pi / 3]  # Angles of points relative to circle (spaced 120deg apart)     


'Parametric equations for circular motion'
def circular_motion(t, phase):
    x = R * np.cos(omega * (t) + phase)  # Rotating x position
    y = R * np.sin(omega * (t) + phase)  # Rotating y position
    z = h                                          # Constant z-coordinate at hover height
    return x, y, z


'Simulation'
positions_over_time = []

total_time = 2*(T)   # total sim time
timestep = 0.1
times = np.arange(0, total_time, timestep)

for t in times:
    positions = [circular_motion(t, phases[i]) for i in range(3)]
    positions_over_time.append(np.array(positions))

positions_over_time = np.array(positions_over_time)


'Real-time 3D Plot'
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for step, t in enumerate(times):
    ax.cla()
    positions = positions_over_time[step]
    x, y, z = positions[:, 0], positions[:, 1], positions[:, 2]
    ax.scatter(x, y, z, c='b', label='Triangle Points')
    for i in range(3):
        x_line = [x[i], x[(i + 1) % 3]]
        y_line = [y[i], y[(i + 1) % 3]]
        z_line = [z[i], z[(i + 1) % 3]]
        ax.plot(x_line, y_line, z_line, c='r')
    
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])
    ax.set_zlim([0, 1.5])
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("Circular Motion in Triangle Formation")
    
    plt.pause(timestep)

plt.show()
