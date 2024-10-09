import logging
import time
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# URI for each drone
URI1 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')  # Sun
URI2 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E2')  # Earth
URI3 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E3')  # Moon

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

def circular_orbit(t, radius, angular_speed):
    x = radius * np.cos(angular_speed * t)
    y = radius * np.sin(angular_speed * t)
    return x, y

def main():
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf_sun, \
         SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf_earth, \
         SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf_moon:

        reset_estimator(scf_sun)
        reset_estimator(scf_earth)
        reset_estimator(scf_moon)

        hlc_sun = scf_sun.cf.high_level_commander
        hlc_earth = scf_earth.cf.high_level_commander
        hlc_moon = scf_moon.cf.high_level_commander

        # Take off all drones to the same height (z=1.0 meters)
        hlc_sun.takeoff(1.0, 2.0)
        hlc_earth.takeoff(1.0, 2.0)
        hlc_moon.takeoff(1.0, 2.0)
        time.sleep(2)

        start_time = time.time()

        try:
            while True:
                t = time.time() - start_time

                # Sun remains static at (0, 0, 1.0)
                hlc_sun.go_to(0.0, 0.0, 1.0, 0, 2.0, relative=False)

                # Earth elliptical orbit around the Sun
                earth_x, earth_y = elliptical_orbit(t, 1.5, 1.0, 0.2)  # a=1.5, b=1.0, angular speed=0.2
                hlc_earth.go_to(earth_x, earth_y, 1.0, 0, 2.0, relative=False)

                # Moon circular orbit around the Earth
                moon_x_offset, moon_y_offset = circular_orbit(t, 0.4, 0.5)  # radius=0.4, angular speed=0.5
                moon_x = earth_x + moon_x_offset
                moon_y = earth_y + moon_y_offset
                hlc_moon.go_to(moon_x, moon_y, 1.0, 0, 2.0, relative=False)

                time.sleep(0.1)

        finally:
            # Land all drones when interrupted
            hlc_sun.land(0.0, 2.0)
            hlc_earth.land(0.0, 2.0)
            hlc_moon.land(0.0, 2.0)
            time.sleep(2)

if __name__ == "__main__":
    main()
