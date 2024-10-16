import numpy as np
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

# Constants and URIs...
# (Your existing constants and imports)

# Global variable to store position
current_position = [0.0, 0.0, 0.0]

# Function to log the current position
def log_position_callback(timestamp, data, logconf):
    global current_position
    current_position = [data['kalman.stateX'], data['kalman.stateY'], data['kalman.stateZ']]

# Function to start logging position
def start_position_logging(scf):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_position_callback)
    log_conf.start()

# Function to get current position
def get_position():
    global current_position
    return current_position

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
        
        # Start logging position for all drones
        start_position_logging(scf_earth)
        start_position_logging(scf_moon)

        earth.takeoff(0.5, 2.0)
        sun.takeoff(0.5, 2.0)
        moon.takeoff(0.5, 2.0)
        time.sleep(2)

        try:
            for i in range(len(x1)):
                # Get current positions of Earth and Moon
                current_earth_position = get_position()
                orbit_x, orbit_y = calculate_orbit_around_point(x1[i], y1[i], orbit_radius, num_points_for_moon)
                current_moon_position = get_position()  # Fetch moon's position

                # Check if within epsilon for both x, y, and z
                if reached_waypoint(current_earth_position, [x1[i], y1[i], 1.5], epsilon) and \
                   within_z_leeway(current_earth_position[2], 1.5, z_epsilon):
                    earth.go_to(x1[(i+1) % len(x1)], y1[(i+1) % len(y1)], 1.5, 0, 1.0, relative=False)

                if reached_waypoint(current_moon_position, [orbit_x[i % num_points_for_moon], orbit_y[i % num_points_for_moon], 1.5], epsilon) and \
                   within_z_leeway(current_moon_position[2], 1.5, z_epsilon):
                    moon.go_to(orbit_x[(i+1) % num_points_for_moon], orbit_y[(i+1) % num_points_for_moon], 1.5, 0, 1.0, relative=False)

                # Move sun to a fixed point
                sun.go_to(0, 0, 1.5, 0, 1.0, relative=False)

                time.sleep(earth_time_interval)

        finally:
            earth.land(0, 2.0)
            moon.land(0, 2.0)
            sun.land(0, 2.0)

if __name__ == "__main__":
    main()
