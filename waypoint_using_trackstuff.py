import numpy as np
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper


# Constants
scale_factor = 10e6  # Scale down distances by this factor
G = 6.67430e-11  # Gravitational constant (m^3 kg^-1 s^-2)
mu = 3.986e14    # Standard gravitational parameter for Earth (m^3 s^-2)
M = 5.97e24      # Mass of Earth (kg)
R = 6371000      # Radius of Earth (m)
r_0 = 500000     # Perigee altitude (m)
v_0 = 9000       # Initial Tangential Velocity of Spacecraft at Perigee (m/s)

# Crazyflie URIs and initialization
URI_EARTH = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
URI_SUN = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI_MOON = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E1')
cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

# Orbital planetary motion simulation calculations
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

def plot_orbit(num_points=1000):
    total_time = 2 * np.pi * np.sqrt(semi_major_axis()**3 / mu)
    t = np.linspace(0, total_time, num_points)
    theta = mean_to_true(t)
    r = keplers_first_law(theta)
    x = r * np.cos(theta) / scale_factor
    y = r * np.sin(theta) / scale_factor
    return x, y

def calculate_orbit_around_point(center_x, center_y, radius, num_points=100):
    angles = np.linspace(0, 2 * np.pi, num_points)
    x = center_x + radius * np.cos(angles)
    y = center_y + radius * np.sin(angles)
    return x, y




# Add epsilon (neighborhood) and z leeway
epsilon = 0.05  # Define the epsilon zone (5 cm radius)
z_epsilon = 0.1  # Z-axis tolerance range (10 cm)

def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

#Check if the current position is within the epsilon range of the target position#
def reached_waypoint(current_pos, target_pos, epsilon):
    return distance(current_pos, target_pos) < epsilon

#Check if current z is within the z leeway range#
def within_z_leeway(current_z, target_z, z_epsilon):
    return abs(current_z - target_z) < z_epsilon

#Drone orbit commands
def main():
    x1, y1 = plot_orbit()

    orbit_radius = 0.5             #orbit radius of moon around earth
    num_points_for_moon = len(x1)

    moon_time_interval = 0.03
    earth_time_interval = 0.07

    with SyncCrazyflie(URI_EARTH, cf=Crazyflie(rw_cache='./cache')) as scf_earth, \
        SyncCrazyflie(URI_SUN, cf=Crazyflie(rw_cache='./cache')) as scf_sun, \
        SyncCrazyflie(URI_MOON, cf=Crazyflie(rw_cache='./cache')) as scf_moon:

        earth = scf_earth.cf.high_level_commander
        sun = scf_sun.cf.high_level_commander
        moon = scf_moon.cf.high_level_commander

        earth.takeoff(0.5, 2.0)
        sun.takeoff(0.5, 2.0)
        moon.takeoff(0.5, 2.0)
        time.sleep(2)

        try:
            for i in range(len(x1)):
                # Calculate the center position for moon's orbit around earth
                center_x = x1[i]
                center_y = y1[i]
                # Calculate positions for moon to orbit around earth at the current point
                orbit_x, orbit_y = calculate_orbit_around_point(center_x, center_y, orbit_radius, num_points_for_moon)
                x3 = orbit_x[i % num_points_for_moon]
                y3 = orbit_y[i % num_points_for_moon]
                
                # Current positions of Earth and Moon
                current_pos_earth = (center_x, center_y, 1.5)
                current_pos_moon = (x3, y3, 1.5)

#waypoints

                # Move Earth if within epsilon and Z leeway
                if reached_waypoint(current_pos_earth[:2], (center_x, center_y), epsilon) and \
                   within_z_leeway(current_pos_earth[2], 1.5, z_epsilon):
                    earth.go_to(x1[(i+1) % len(x1)], y1[(i+1) % len(y1)], 1.5, 0, 1.0, relative=False)

                # Move Moon if within epsilon and Z leeway
                if reached_waypoint(current_pos_moon[:2], (x3, y3), epsilon) and \
                   within_z_leeway(current_pos_moon[2], 1.5, z_epsilon):
                    moon.go_to(orbit_x[(i+1) % num_points_for_moon], orbit_y[(i+1) % num_points_for_moon], 1.5, 0, 1.0, relative=False)

                # Move sun to a fixed point
                sun.go_to(0, 0, 1.5, 0, 1.0, relative=False)

                time.sleep(earth_time_interval)
                print(f'earth moving to x={center_x:.2f}, y={center_y:.2f}')
                print(f'moon orbiting around earth at x={x3:.2f}, y={y3:.2f}')

        finally:
            earth.land(0, 2.0)
            sun.land(0, 2.0)
            moon.land(0, 2.0)
            print("Landing...")

if __name__ == "__main__":
    main()