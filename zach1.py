import logging
import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# URI for each drone (Sun and Earth)
URI_SUN = 'radio://0/20/2M/E7E7E7E7E1'
URI_EARTH = 'radio://0/80/2M/E7E7E7E7E1'

cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

def elliptical_orbit(t, semi_major, semi_minor, angular_speed):
    x = semi_major * np.cos(angular_speed * t)
    y = semi_minor * np.sin(angular_speed * t)
    return x, y

def main():
    with SyncCrazyflie(URI_SUN, cf=Crazyflie(rw_cache='./cache')) as scf_sun, \
         SyncCrazyflie(URI_EARTH, cf=Crazyflie(rw_cache='./cache')) as scf_earth:

        reset_estimator(scf_sun)
        reset_estimator(scf_earth)

        hlc_sun = scf_sun.cf.high_level_commander
        hlc_earth = scf_earth.cf.high_level_commander

        # Take off both drones to the same height (z=1.0 meters)
        hlc_sun.takeoff(1.0, 2.0)
        hlc_earth.takeoff(1.0, 2.0)
        time.sleep(2)

        start_time = time.time()

        try:
            while True:
                t = time.time() - start_time

                # Sun remains static at (0, 0, 1.0)
                hlc_sun.go_to(0.0, 0.0, 1.0, 0, 2.0, relative=False)

                # Earth elliptical orbit around the Sun
                earth_x, earth_y = elliptical_orbit(t, 2.0, 1.5, 0.1)  # a=2.0, b=1.5, angular speed=0.1
                hlc_earth.go_to(earth_x, earth_y, 1.0, 0, 2.0, relative=False)

                time.sleep(0.1)

        finally:
            # Land both drones when interrupted
            hlc_sun.land(0.0, 2.0)
            hlc_earth.land(0.0, 2.0)
            time.sleep(2)

if __name__ == "__main__":
    main()
