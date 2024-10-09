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
R = 6371000      # Radius of Earth (meters)
r_0 = 500000     # Perigee altitude (meters)
v_0 = 9000       # Initial Tangential Velocity of Spacecraft at Perigee (m/s)

# Crazyflie URIs and initialization
URI1 = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
URI2 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI3 = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E1')
cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

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

def main():
    # Generate the orbital path for hlc1
    x1, y1 = plot_orbit()

    # Define the radius of the orbit for hlc3 around hlc1
    orbit_radius = 0.5  # in meters
    num_points_for_hlc3 = len(x1)  # match the points to keep them in sync

    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1, \
        SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2, \
        SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:
        
        hlc1 = scf1.cf.high_level_commander
        hlc2 = scf2.cf.high_level_commander
        hlc3 = scf3.cf.high_level_commander
        
        hlc1.takeoff(0.5, 1.0)
        hlc2.takeoff(0.5, 1.0)
        hlc3.takeoff(0.5, 1.0)
        time.sleep(2)

        try:
            time_interval = 0.1
            for i in range(len(x1)):
                # Calculate the center position for hlc3's orbit around hlc1
                center_x = x1[i]
                center_y = y1[i]
                
                # Calculate positions for hlc3 to orbit around hlc1 at the current point
                orbit_x, orbit_y = calculate_orbit_around_point(center_x, center_y, orbit_radius, num_points_for_hlc3)
                x3 = orbit_x[i % num_points_for_hlc3]
                y3 = orbit_y[i % num_points_for_hlc3]

                # Move hlc1 along its orbital path
                hlc1.go_to(center_x, center_y, 1.5, 0, 1.0, relative=False)
                # Move hlc2 to a fixed point or in a different path
                hlc2.go_to(0, 0, 1.5, 0, 1.0, relative=False)
                # Move hlc3 around hlc1's current position
                hlc3.go_to(x3, y3, 1.5, 0, 1.0, relative=False)

                time.sleep(time_interval)
                print(f'hlc1 moving to x={center_x:.2f}, y={center_y:.2f}')
                print(f'hlc3 orbiting around hlc1 at x={x3:.2f}, y={y3:.2f}')

        finally:
            hlc1.land(0, 2.0)
            hlc2.land(0, 2.0)
            hlc3.land(0, 2.0)
            print("Landing...")

if __name__ == "__main__":
    main()