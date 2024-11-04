import numpy as np
import time
import csv

# Constants
scale_factor = 10e10  # Scale down distances by this factor
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
mu = 1.327e20    # Standard gravitational parameter for Sun (m^3 s^-2)
R = 6.96e8     # Radius of Sun (meters)
r_0 = 147100000000     # Perigee altitude (meters)
v_0 = 30290     # Initial Tangential Velocity of Spacecraft at Perigee (m/s)


def specific_orbital_energy():
    soe = ((v_0**2) / 2) - (mu / (R + r_0))
    return soe

def semi_major_axis():
    a = -(mu / (2 * specific_orbital_energy()))
    return a

def specific_angular_momentum():
    h = (R + r_0) * v_0
    return h

def eccentricity():
    e = np.sqrt(1 + (2 * specific_orbital_energy() * specific_angular_momentum()**2) / (mu**2))
    return e

def keplers_first_law(theta):
    a = semi_major_axis()
    e = eccentricity()
    r = a * (1 - e**2) / (1 + e * np.cos(theta))
    return r

def mean_motion():
    a = semi_major_axis()
    n = np.sqrt(mu / a**3)
    return n

def mean_to_true(time_in_seconds):
    n = mean_motion()
    e = eccentricity()
    m = n * np.asarray(time_in_seconds)
    m = m % (2 * np.pi)
    E = m + e * np.sin(m) / (1 - np.sin(m + e) + np.sin(m))
    for _ in range(10):
        E = E - (E - e * np.sin(E) - m) / (1 - e * np.cos(E))
    f = 2 * np.arctan(np.sqrt((1 + e) / (1 - e)) * np.tan(E / 2))
    f = f % (2 * np.pi)
    return f

def plot_orbit(num_points=100):
    total_time = 2 * np.pi * np.sqrt(semi_major_axis()**3 / mu)
    t = np.linspace(0, total_time, num_points)
    t_scaled = np.linspace(0, total_time/1067859.03756, num_points)
    theta = mean_to_true(t)
    r = keplers_first_law(theta)
    x = r * np.cos(theta) / scale_factor
    y = r * np.sin(theta) / scale_factor
    z = 1.0
    yaw = 0
    return t_scaled, x, y, z, yaw

def calculate_orbit_around_point(center_x, center_y, radius, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return x, y

def main():
    # Generate the orbital path for Earth
    t1, x1, y1, z1, yaw1 = plot_orbit()

    # Specify the output CSV file name
    output_filename = 'orbital_path.csv'
    
    # Write the x1, y1 pairs to a CSV file
    with open(output_filename, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['t', 'x', 'y', 'z', 'yaw'])  # Write header
        for t, x, y, z_value, yaw_value in zip(t1, x1, y1, [z1] * len(x1), [yaw1] * len(y1)):
            writer.writerow([t, x, y, z_value, yaw_value])  # Write each pair

    #print(f'Orbital path data written to {output_filename}')

if __name__ == '__main__':
    main()
