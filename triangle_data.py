import numpy as np
import time
import csv

# Constants
L = 0.5                                     # Side length of the equilateral triangle [m]
R = L / np.sqrt(3)                          # Radius of the circular motion [m]
T = 10                                      # Period of rotation [s]
h = 1.0                                     # Hover height [m]
t_hover = 5                                 # Time for upward motion [s]
omega = 2 * np.pi / T                       # Angular velocity [rad/s]
phases = [0, 2 * np.pi / 3, 4 * np.pi / 3]  # Angles of points relative to circle (spaced 120 degrees apart)   
total_time = t_hover + T                    # Total simulation time
num_points = 100

def vertical_motion(t, phase):
    x = R * np.cos(phase)
    y = R * np.sin(phase)
    z = t * h / t_hover 
    return x, y, z

def circular_motion(t, phase):
    x = R * np.cos(omega * (t - t_hover) + phase)
    y = R * np.sin(omega * (t - t_hover) + phase)
    z = h 
    return x, y, z


def point_path(phase):
    t = np.linspace(0, total_time, num_points)
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.zeros_like(t)
    yaw = np.zeros_like(t)

    for i, t_val in enumerate(t):
        if t_val <= t_hover:
            x[i], y[i], z[i] = vertical_motion(t_val, phase)
        else:
            x[i], y[i], z[i] = circular_motion(t_val, phase)
        yaw[i] = 0  # constant yaw
    return t, x, y, z, yaw


def main():
    # Generate paths for each point/drone
    t1, x1, y1, z1, yaw1 = point_path(phases[0])   #drone 1 @ 0 degrees
    t2, x2, y2, z2, yaw2 = point_path(phases[1])   #drone 2 @ 120 degrees
    t3, x3, y3, z3, yaw3 = point_path(phases[2])   #drone 3 @ 240 degrees

    point1_filename = 'point1_path.csv'
    point2_filename = 'point2_path.csv'
    point3_filename = 'point3_path.csv'

    # Write paths to CSV files
    with open(point1_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['t', 'x', 'y', 'z', 'yaw']) 
        for t, x, y, z_value, yaw_value in zip(t1, x1, y1, z1, yaw1):
            writer.writerow([t, x, y, z_value, yaw_value]) 

    with open(point2_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['t', 'x', 'y', 'z', 'yaw']) 
        for t, x, y, z_value, yaw_value in zip(t2, x2, y2, z2, yaw2):
            writer.writerow([t, x, y, z_value, yaw_value]) 

    with open(point3_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['t', 'x', 'y', 'z', 'yaw']) 
        for t, x, y, z_value, yaw_value in zip(t3, x3, y3, z3, yaw3):
            writer.writerow([t, x, y, z_value, yaw_value]) 

    print(f'Paths written to CSV files: {point1_filename}, {point2_filename}, {point3_filename}')

if __name__ == '__main__':
    main()
