import logging
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from cvzone.HandTrackingModule import HandDetector
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

URI = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')

# Initialize Crazyflie
cflib.crtp.init_drivers()

# Constants for scaled-down "orbit"
semi_major_axis = 0.75  # 0.75 meters for 1.5m space
eccentricity = 0.2  # You can adjust this to change orbit shape
num_points = 200  # Number of points in the orbit
total_time = 20  # Total time for one orbit in seconds
dt = total_time / num_points  # Time step for each point

# Calculate the orbit points
def calculate_orbit_points():
    theta = np.linspace(0, 2 * np.pi, num_points)
    r = semi_major_axis * (1 - eccentricity**2) / (1 + eccentricity * np.cos(theta))
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def main():
    x_orbit, y_orbit = calculate_orbit_points()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        mc = MotionCommander(scf)

        # Takeoff
        mc.take_off(0.5, 2.0)  # Take off to 0.5 meters
        time.sleep(2)

        # Go to the starting position (0, 0, 1.5)
        mc.go_to(0, 0, 1.5, velocity=0.2)
        time.sleep(2)  # Pause to stabilize at (0, 0, 1.5)

        try:
            # Start the orbiting movement
            for i in range(num_points):
                # Move to the next point on the orbit (scaled-down to 1.5m space)
                mc.go_to(x_orbit[i], y_orbit[i], 1.5, velocity=0.2)  # Altitude is kept at 1.5 meters
                time.sleep(dt)  # Pause for a moment before moving to the next point

        finally:
            # Land after completing the orbit
            mc.land()

if __name__ == "__main__":
    main()