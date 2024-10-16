import numpy as np
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

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

# Initialize global variables for positions
current_position_earth = [0.0, 0.0, 0.0]
current_position_moon = [0.0, 0.0, 0.0]

# Logging callbacks to update the drone positions
def log_position_earth_callback(timestamp, data, logconf):
    global current_position_earth
    current_position_earth = [data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']]

def log_position_moon_callback(timestamp, data, logconf):
    global current_position_moon
    current_position_moon = [data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']]

# Function to set up position logging
def start_position_logging(scf, callback):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(callback)
    log_conf.start()


# Add epsilon (neighborhood) and z leeway
epsilon = 0.2  # Distance threshold to transition to next waypoint (neighborhood distance)
z_epsilon = 0.05  # Allowed variation in z-axis (Z leeway)

def reached_waypoint(current_position, target_position, epsilon):
    """Check if the current position is within the epsilon range of the target position."""
    distance = np.linalg.norm(np.array(current_position[:2]) - np.array(target_position[:2]))
    return distance < epsilon

def within_z_leeway(current_z, target_z, z_epsilon):
    """Check if current z is within the z leeway range."""
    return abs(current_z - target_z) < z_epsilon

def main():

    # Generate the orbital path for earth
    x1, y1 = plot_orbit()

    # Define the radius of the orbit for moon around earth
    orbit_radius = 0.5  # in meters
    num_points_for_moon = len(x1)  # Match the points for synchronization 

    # Make the moon move faster by using a shorter time interval for its orbit
    moon_time_interval = 0.03  # Smaller interval for faster movement
    earth_time_interval = 0.07  # Interval for earth's movement

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
                # Current positions of Earth and Moon
                current_earth_position = [x1[i], y1[i], 1.5]
                orbit_x, orbit_y = calculate_orbit_around_point(x1[i], y1[i], orbit_radius, num_points_for_moon)
                current_moon_position = [orbit_x[i % num_points_for_moon], orbit_y[i % num_points_for_moon], 1.5]

                # Check if within epsilon for both x,y, and z
                if reached_waypoint(earth.get_position(), current_earth_position, epsilon) and \
                   within_z_leeway(earth.get_position()[2], 1.5, z_epsilon):
                    earth.go_to(x1[(i+1) % len(x1)], y1[(i+1) % len(y1)], 1.5, 0, 1.0, relative=False)

                if reached_waypoint(moon.get_position(), current_moon_position, epsilon) and \
                   within_z_leeway(moon.get_position()[2], 1.5, z_epsilon):
                    moon.go_to(orbit_x[(i+1) % num_points_for_moon], orbit_y[(i+1) % num_points_for_moon], 1.5, 0, 1.0, relative=False)

                # Move sun to a fixed point
                sun.go_to(0, 0, 1.5, 0, 1.0, relative=False)

                time.sleep(earth_time_interval)

        finally:
            earth.land(0, 2.0)
            sun.land(0, 2.0)
            moon.land(0, 2.0)

if __name__ == "__main__":
    main()


