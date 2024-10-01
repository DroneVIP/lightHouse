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

URI1 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')
cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)


def position_callback(timestamp, data, logconf):
    x = data['kalman.stateX']
    y = data['kalman.stateY']
    z = data['kalman.stateZ']
    print(f'Position: x={x:.2f}, y={y:.2f}, z={z:.2f}')

def start_position_logging(scf):
    log_conf = LogConfig(name='Position', period_in_ms=200)
    log_conf.add_variable('kalman.stateX', 'float')
    log_conf.add_variable('kalman.stateY', 'float')
    log_conf.add_variable('kalman.stateZ', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

def main():
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1:
        reset_estimator(scf1)
        start_position_logging(scf1)
        hlc1 = scf1.cf.high_level_commander
        hlc1.takeoff(0.5, 2.0)
        time.sleep(1)

        try:
            while True:
                time.sleep(1)
                hlc1.go_to(-1.53, -0.05, 1.50, 0, 3, relative=False)

        finally:
            hlc1.land(0, 2.0)
if __name__ == "__main__":
    main()