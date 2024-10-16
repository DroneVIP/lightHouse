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
tolerance = 0.05  # Acceptable distance error in meters

# PID controller constants
Kp = 1.0  # Proportional gain
Ki = 0.0  # Integral gain
Kd = 0.5  # Derivative gain

# Crazyflie URIs and initialization
URI_EARTH = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
URI_SUN = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
URI_MOON = uri_helper.uri_from_env(default='radio://0/60/2M/E7E7E7E7E1')
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

# PID control function to adjust speed as the drone approaches the target
def pid_control(current_position, target_position, previous_error, integral, delta_time):
    error = target_position - current_position
    derivative = (error - previous_error) / delta_time
    integral += error * delta_time
    control_value = (Kp * error) + (Ki * integral) + (Kd * derivative)
    return control_value, error, integral

def main():
    # Generate the orbital path for earth
    x1, y1 = plot_orbit()

    # Define the radius of the orbit for moon around earth
    orbit_radius = 0.5  # in meters
    num_points_for_moon = len(x1)  # Match the points for synchronization

    moon_time_interval = 0.05
    earth_time_interval = 0.1

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

        previous_error_earth_x = previous_error_earth_y = 0
        previous_error_moon_x = previous_error_moon_y = 0
        integral_earth_x = integral_earth_y = 0
        integral_moon_x = integral_moon_y = 0

        try:
            for i in range(len(x1)):
                center_x = x1[i]
                center_y = y1[i]
                
                orbit_x, orbit_y = calculate_orbit_around_point(center_x, center_y, orbit_radius, num_points_for_moon)
                x3 = orbit_x[i % num_points_for_moon]
                y3 = orbit_y[i % num_points_for_moon]

                # Apply PID control for smoother movement (Earth)
                control_x, error_x, integral_earth_x = pid_control(
                    current_position=scf_earth.cf.position_estimate[0],
                    target_position=center_x,
                    previous_error=previous_error_earth_x,
                    integral=integral_earth_x,
                    delta_time=earth_time_interval)
                
                control_y, error_y, integral_earth_y = pid_control(
                    current_position=scf_earth.cf.position_estimate[1],
                    target_position=center_y,
                    previous_error=previous_error_earth_y,
                    integral=integral_earth_y,
                    delta_time=earth_time_interval)

                previous_error_earth_x = error_x
                previous_error_earth_y = error_y

                # Move drones based on PID control
                if abs(error_x) > tolerance or abs(error_y) > tolerance:
                    earth.go_to(center_x + control_x, center_y + control_y, 1.5, 0, 1.0, relative=False)
                sun.go_to(0, 0, 1.5, 0, 1.0, relative=False)
                moon.go_to(x3, y3, 1.5, 0, 1.0, relative=False)

                time.sleep(earth_time_interval)
                print(f'Earth moving to x={center_x:.2f}, y={center_y:.2f}')
                print(f'Moon orbiting around Earth at x={x3:.2f}, y={y3:.2f}')

        finally:
            earth.land(0, 2.0)
            sun.land(0, 2.0)
            moon.land(0, 2.0)
            print("Landing...")

if __name__ == "__main__":
    main()
