import logging
import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# URI for each drone (Sun, Earth, Moon)
URI_SUN = 'radio://0/20/2M/E7E7E7E7E1'
URI_EARTH = 'radio://0/80/2M/E7E7E7E7E1'
URI_MOON = 'radio://0/30/2M/E7E7E7E7E3'

cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

# 타원의 한 초점에 태양이 있는 경우 반영
def elliptical_orbit_with_offset(t, semi_major, semi_minor, angular_speed, offset_x):
    x = semi_major * np.cos(angular_speed * t) + offset_x  # 태양이 타원의 중심에서 오른쪽으로 치우친 부분
    y = semi_minor * np.sin(angular_speed * t)
    return x, y

def circular_orbit(t, radius, angular_speed):
    x = radius * np.cos(angular_speed * t)
    y = radius * np.sin(angular_speed * t)
    return x, y

def main():
    with SyncCrazyflie(URI_SUN, cf=Crazyflie(rw_cache='./cache')) as scf_sun, \
         SyncCrazyflie(URI_EARTH, cf=Crazyflie(rw_cache='./cache')) as scf_earth, \
         SyncCrazyflie(URI_MOON, cf=Crazyflie(rw_cache='./cache')) as scf_moon:

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
                sun_x, sun_y = 0.0, 0.0
                hlc_sun.go_to(sun_x, sun_y, 1.0, 0, 2.0, relative=False)

                # Earth elliptical orbit around the Sun with the Sun offset from the center
                offset_x = 0.5  # Offset to shift the sun away from the center of the ellipse
                earth_x, earth_y = elliptical_orbit_with_offset(t, 1.0, 0.75, 0.05, offset_x)  # a=1.0, b=0.75, angular speed=0.05
                hlc_earth.go_to(earth_x, earth_y, 1.0, 0, 2.0, relative=False)

                # Moon circular orbit around the Earth
                moon_x_offset, moon_y_offset = circular_orbit(t, 0.3, 0.2)  # radius=0.3, slower angular speed=0.2
                moon_x = earth_x + moon_x_offset
                moon_y = earth_y + moon_y_offset
                hlc_moon.go_to(moon_x, moon_y, 1.0, 0, 2.0, relative=False)

                # Print positions
                print(f"Sun: x={sun_x:.2f}, y={sun_y:.2f}")
                print(f"Earth: x={earth_x:.2f}, y={earth_y:.2f}")
                print(f"Moon: x={moon_x:.2f}, y={moon_y:.2f}")

                time.sleep(0.1)

        finally:
            # Land all drones when interrupted
            hlc_sun.land(0.0, 2.0)
            hlc_earth.land(0.0, 2.0)
            hlc_moon.land(0.0, 2.0)
            time.sleep(2)

if __name__ == "__main__":
    main()
