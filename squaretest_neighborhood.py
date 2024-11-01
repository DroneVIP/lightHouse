import numpy as np
import time
import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.crazyflie.log import LogConfig

#Initializing
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

current_position = (0, 0, 0)

#Position logging
def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    current_position = [x, y, z]

def start_position_logging(scf):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

#def get_position(drone_name):
 #   return current_position[drone_name]

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

epsilon = 0.2  # Neighborhood distance in x-y plane
z_epsilon = 0.05  # Allowed variation in z-axis

def reached_waypoint(current_position, target_position, epsilon):
    """Check if the current position is within the epsilon range of the target position."""
    distance = np.linalg.norm(np.array(current_position[:2]) - np.array(target_position[:2]))
    return distance < epsilon

def within_z_leeway(current_z, target_z, z_epsilon):
    """Check if current z is within the z leeway range."""
    return abs(current_z - target_z) < z_epsilon

def main():

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        reset_estimator(scf)
        start_position_logging(scf)
        hlc = scf.cf.high_level_commander

        hlc.takeoff(1.0, 5)
        time.sleep(5)

        # Define waypoints for the square trajectory
        waypoints = [
        (0.5, 0.0, 1.0),
        (0.5, 0.5, 1.0),
        (0.0, 0.5, 1.0),
        (0.0, 0.0, 1.0),
        ]

        # Fly through each waypoint tragectory
        for waypoint in waypoints:
            target_x, target_y, target_z = 3 * waypoint[0], 3 * waypoint[1], waypoint[2]
            hlc.go_to(target_x, target_y, target_z, 0, 5)

            # Wait until waypoint is reached

            while True:
                # Check if within epsilon and z leeway
                if (reached_waypoint(current_position, (target_x, target_y, target_z), epsilon) and within_z_leeway(current_position[2], target_z, z_epsilon)):
                    print(f"Reached waypoint: x={target_x}, y={target_y}, z={target_z}")
                    break
            time.sleep(1)

            hlc.land(0.0, 5)

if __name__ == "__main__":
    main()
