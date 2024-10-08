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

import numpy as np
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.high_level_commander import HighLevelCommander

URI = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')

# Initialize Crazyflie
cflib.crtp.init_drivers()

# Constants for scaled-down "orbit"
semi_major_axis = 0.75  # 0.75 meters for 1.5m space
eccentricity = 0.2  # You can adjust this to change orbit shape
num_points = 400  # Number of points in the orbit
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
        hlc = HighLevelCommander(scf.cf)

        # Takeoff to 0.5m and hover for 2 seconds
        hlc.takeoff(0.5, 2.0)
        time.sleep(2)

        # Move to the starting position (0, 0, 1.5)
        hlc.go_to(0, 0, 1.5, 0, 2.0)  # Absolute coordinates with yaw=0
        time.sleep(2)  # Pause to stabilize at (0, 0, 1.5)

        try:
            # Start the orbiting movement
            for i in range(num_points):
                # Move to the next point on the orbit (scaled-down to 1.5m space)
                hlc.go_to(x_orbit[i], y_orbit[i], 1.5, 0, dt)  # Altitude is kept at 1.5 meters, yaw=0

        finally:
            # Land after completing the orbit
            hlc.land(0.0, 2.0)

if __name__ == "__main__":
    main()
