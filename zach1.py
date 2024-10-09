import logging
import time
import numpy as np
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.positioning.motion_commander import MotionCommander

# URI for each drone
URI1 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E1')  # Sun
URI2 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E2')  # Earth
URI3 = uri_helper.uri_from_env(default='radio://0/30/2M/E7E7E7E7E3')  # Moon

cflib.crtp.init_drivers()
logging.basicConfig(level=logging.ERROR)

# Helper functions
def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)

def go_to_position(mc, x, y, z, speed=0.3):
    mc.move_to(x, y, z, velocity=speed)

def circular_orbit(time, radius, angular_speed):
    x = radius * np.cos(angular_speed * time)
    y = radius * np.sin(angular_speed * time)
    return x, y

def elliptical_orbit(time, semi_major, semi_minor, angular_speed):
    x = semi_major * np.cos(angular_speed * time)
    y = semi_minor * np.sin(angular_speed * time)
    return x, y

def main():
    # Sun (URI1) stays static, Earth (URI2) moves in an elliptical orbit, Moon (URI3) orbits Earth in a circular orbit
    with SyncCrazyflie(URI1, cf=Crazyflie(rw_cache='./cache')) as scf1, \
         SyncCrazyflie(URI2, cf=Crazyflie(rw_cache='./cache')) as scf2, \
         SyncCrazyflie(URI3, cf=Crazyflie(rw_cache='./cache')) as scf3:

        reset_estimator(scf1)
        reset_estimator(scf2)
        reset_estimator(scf3)

        mc_sun = MotionCommander(scf1)  # Sun stays static
        mc_earth = MotionCommander(scf2)  # Earth motion
        mc_moon = MotionCommander(scf3)  # Moon motion

        try:
            # Take off all drones to the same height (1.0 meters)
            mc_sun.take_off(1.0)
            mc_earth.take_off(1.0)
            mc_moon.take_off(1.0)
            time.sleep(2)

            # Align drones on x-axis: Sun at x=0, Earth at x=1.5, Moon at x=1.9
            go_to_position(mc_sun, 0.0, 0.0, 1.0)
            go_to_position(mc_earth, 1.5, 0.0, 1.0)
            go_to_position(mc_moon, 1.9, 0.0, 1.0)
            time.sleep(2)

            start_time = time.time()

            while True:
                t = time.time() - start_time

                # Earth elliptical orbit around the Sun
                earth_x, earth_y = elliptical_orbit(t, 1.5, 1.0, 0.2)  # a=1.5, b=1.0, angular speed=0.2
                go_to_position(mc_earth, earth_x, earth_y, 1.0)

                # Moon circular orbit around the Earth
                moon_x_offset, moon_y_offset = circular_orbit(t, 0.4, 1.0)  # radius=0.4, angular speed=1.0
                moon_x = earth_x + moon_x_offset
                moon_y = earth_y + moon_y_offset
                go_to_position(mc_moon, moon_x, moon_y, 1.0)

                time.sleep(0.1)

        finally:
            # Land all drones when interrupted
            mc_sun.land()
            mc_earth.land()
            mc_moon.land()

if __name__ == "__main__":
    main()
