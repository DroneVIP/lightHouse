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
URI_SUN = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI_EARTH = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
URI_MOON = uri_helper.uri_from_env(default='radio://0/20/2M/E7E7E7E7E1')
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


current_positions = {
    'earth': [0.0, 0.0, 0.0],
    'sun': [0.0, 0.0, 0.0],
    'moon': [0.0, 0.0, 0.0]
}

def position_callback(timestamp, data, logconf, drone_name):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    current_positions[drone_name] = [x, y, z]

def start_position_logging(scf, drone_name):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda timestamp, data, logconf: position_callback(timestamp, data, logconf, drone_name))
    log_conf.start()

def get_position(drone_name):
    return current_positions[drone_name]

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)


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

    earth_time_interval = 0.07  # Interval for earth's movement

    with SyncCrazyflie(URI_EARTH, cf=Crazyflie(rw_cache='./cache')) as scf_earth, \
        SyncCrazyflie(URI_SUN, cf=Crazyflie(rw_cache='./cache')) as scf_sun, \
        SyncCrazyflie(URI_MOON, cf=Crazyflie(rw_cache='./cache')) as scf_moon:

        reset_estimator(scf_earth)
        reset_estimator(scf_sun)
        reset_estimator(scf_moon)
        start_position_logging(scf_earth, 'earth')
        start_position_logging(scf_sun, 'sun')
        start_position_logging(scf_moon, 'moon')

        earth = scf_earth.cf.high_level_commander
        sun = scf_sun.cf.high_level_commander
        moon = scf_moon.cf.high_level_commander

        earth.takeoff(1.5, 10.0)
        sun.takeoff(1.5, 10.0)
        moon.takeoff(1.5, 10.0)
        time.sleep(10)

        try:
            for i in range(len(x1)):
                # Current positions of Earth and Moon
                current_earth_position = get_position('earth')
                orbit_x, orbit_y = calculate_orbit_around_point(x1[i], y1[i], orbit_radius, num_points_for_moon)
                current_moon_position = [orbit_x[i % num_points_for_moon], orbit_y[i % num_points_for_moon], 1.5]
                print(current_earth_position, current_earth_position)

                # Check if within epsilon for both x,y, and z
                if reached_waypoint(current_earth_position, [x1[i], y1[i], 1.5], epsilon) and \
                   within_z_leeway(current_earth_position[2], 1.5, z_epsilon):
                    earth.go_to(x1[(i+1) % len(x1)], y1[(i+1) % len(y1)], 1.5, 0, 1.0, relative=False)

                if reached_waypoint(get_position('moon'), current_moon_position, epsilon) and \
                   within_z_leeway(get_position('moon')[2], 1.5, z_epsilon):
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
