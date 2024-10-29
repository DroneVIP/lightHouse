import numpy as np
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

URI= uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
cflib.crtp.init_drivers()




# Connect to the Crazyflie
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    # Take off
    cf.high_level_commander.takeoff(1.0, 5)
    time.sleep(5)

    # Define the waypoints for the square trajectory
    waypoints = [
        (0.5, 0.0, 1.0),
        (0.5, 0.5, 1.0),
        (0.0, 0.5, 1.0),
        (0.0, 0.0, 1.0),
    ]

    # Fly the trajectory
    for waypoint in waypoints:
        cf.high_level_commander.go_to(3*waypoint[0], 3*waypoint[1], waypoint[2], 0, 5)
        time.sleep(5)

    # Land
    cf.high_level_commander.land(0.0, 5)
    time.sleep(3)