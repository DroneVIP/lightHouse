import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
import concurrent.futures

from cflib.utils import uri_helper

URI_DRONE1 = 'radio://0/10/2M/E7E7E7E7E1'
URI_DRONE2 = 'radio://0/20/2M/E7E7E7E7E1'
URI_DRONE3 = 'radio://0/30/2M/E7E7E7E7E1'
URI_DRONE4 = 'radio://0/50/2M/E7E7E7E7E1'
URI_DRONE5 = 'radio://0/60/2M/E7E7E7E7E1'
URI_DRONE6 = 'radio://0/80/2M/E7E7E7E7E1'

drone1_trajectory = [
    # [duration, x^0 ... x^7, y^0 ... y^7, z^0 ... z^7, yaw^0 ... yaw^7]
    # 이하 생략 (기존 코드와 동일)
    [2.0, 0.396246, 0.0, -0.0, -0.131152, 0.03983, 0.043832, -0.030239, 0.005263,
     0.028552, 0.0, -0.0, 1.717674, -3.106154, 2.265862, -0.758383, 0.096433, 0.999708, -0.0, -0.0, 0.004137, -0.003961, -0.00047, 0.001324, -0.000308,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    # 나머지 trajectory 데이터는 원문 코드와 동일하게 유지
]

drone2_trajectory = [
    # 원문 코드와 동일
]

drone3_trajectory = [
    # 원문 코드와 동일
]

drone4_trajectory = [
    # 원문 코드와 동일
]

drone5_trajectory = [
    # 원문 코드와 동일
]

drone6_trajectory = [
    # 원문 코드와 동일
]


class Uploader:
    def __init__(self):
        self._is_done = False

    def upload(self, trajectory_mem):
        print('Uploading data')
        trajectory_mem.write_data(self._upload_done)

        while not self._is_done:
            time.sleep(0.2)

    def _upload_done(self, mem, addr):
        print('Data uploaded')
        self._is_done = True


def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)


def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.poly4Ds.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    Uploader().upload(trajectory_mem)
    cf.high_level_commander.define_trajectory(trajectory_id, 0,
                                              len(trajectory_mem.poly4Ds))
    return total_duration


def set_led_color(cf, red, green, blue):
    # ring.effect=7: solid color
    cf.param.set_value('ring.effect', '7')
    cf.param.set_value('ring.solidRed', str(red))
    cf.param.set_value('ring.solidGreen', str(green))
    cf.param.set_value('ring.solidBlue', str(blue))


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    # 처음에 LED를 초록색으로 설정
    set_led_color(cf, 0, 100, 0)  # Green

    # Trajectory 시작
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)

    start_time = time.time()
    color_green = True  # True면 초록, False면 빨강

    # duration 동안 1초마다 LED 색상 토글
    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration:
            break
        # 1초 간격
        time.sleep(1)
        color_green = not color_green
        if color_green:
            set_led_color(cf, 0, 100, 0)   # Green
        else:
            set_led_color(cf, 100, 0, 0)  # Red

    # Trajectory 종료 후 착륙
    commander.land(0.0, 5.0)
    time.sleep(5)
    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI_DRONE1, cf=Crazyflie(rw_cache='./cache')) as scf_drone1, \
         SyncCrazyflie(URI_DRONE2, cf=Crazyflie(rw_cache='./cache')) as scf_drone2, \
         SyncCrazyflie(URI_DRONE3, cf=Crazyflie(rw_cache='./cache')) as scf_drone3, \
         SyncCrazyflie(URI_DRONE4, cf=Crazyflie(rw_cache='./cache')) as scf_drone4, \
         SyncCrazyflie(URI_DRONE5, cf=Crazyflie(rw_cache='./cache')) as scf_drone5, \
         SyncCrazyflie(URI_DRONE6, cf=Crazyflie(rw_cache='./cache')) as scf_drone6:

        drone1 = scf_drone1.cf
        drone2 = scf_drone2.cf
        drone3 = scf_drone3.cf
        drone4 = scf_drone4.cf
        drone5 = scf_drone5.cf
        drone6 = scf_drone6.cf

        drone1_trajectory_id = 1
        drone2_trajectory_id = 2
        drone3_trajectory_id = 3
        drone4_trajectory_id = 4
        drone5_trajectory_id = 5
        drone6_trajectory_id = 6

        activate_high_level_commander(drone1)
        activate_high_level_commander(drone2)
        activate_high_level_commander(drone3)
        activate_high_level_commander(drone4)
        activate_high_level_commander(drone5)
        activate_high_level_commander(drone6)

        reset_estimator(drone1)
        reset_estimator(drone2)
        reset_estimator(drone3)
        reset_estimator(drone4)
        reset_estimator(drone5)
        reset_estimator(drone6)

        # 트래젝토리 업로드
        drone1_duration = upload_trajectory(drone1, drone1_trajectory_id, drone1_trajectory)
        drone2_duration = upload_trajectory(drone2, drone2_trajectory_id, drone2_trajectory)
        drone3_duration = upload_trajectory(drone3, drone3_trajectory_id, drone3_trajectory)
        drone4_duration = upload_trajectory(drone4, drone4_trajectory_id, drone4_trajectory)
        drone5_duration = upload_trajectory(drone5, drone5_trajectory_id, drone5_trajectory)
        drone6_duration = upload_trajectory(drone6, drone6_trajectory_id, drone6_trajectory)

        print('The drone1 sequence is {:.1f} seconds long'.format(drone1_duration))
        print('The drone2 sequence is {:.1f} seconds long'.format(drone2_duration))
        print('The drone3 sequence is {:.1f} seconds long'.format(drone3_duration))
        print('The drone4 sequence is {:.1f} seconds long'.format(drone4_duration))
        print('The drone5 sequence is {:.1f} seconds long'.format(drone5_duration))
        print('The drone6 sequence is {:.1f} seconds long'.format(drone6_duration))

        # 이륙 및 초기 위치 이동
        hlc_drone1 = drone1.high_level_commander
        hlc_drone2 = drone2.high_level_commander
        hlc_drone3 = drone3.high_level_commander
        hlc_drone4 = drone4.high_level_commander
        hlc_drone5 = drone5.high_level_commander
        hlc_drone6 = drone6.high_level_commander

        hlc_drone1.takeoff(1, 5.0)
        hlc_drone2.takeoff(1, 5.0)
        hlc_drone3.takeoff(1, 5.0)
        hlc_drone4.takeoff(0.5, 5.0)
        hlc_drone5.takeoff(0.5, 5.0)
        hlc_drone6.takeoff(0.5, 5.0)
        time.sleep(5)

        hlc_drone1.go_to(0.4,0.0,1.0, 0, 5.0, relative=False)
        hlc_drone2.go_to(-0.2,0.3464101615,1.0, 0, 5.0, relative=False)
        hlc_drone3.go_to(-0.2,-0.3464101615,1.0, 0, 5.0, relative=False)
        hlc_drone4.go_to(0.8,0.0,0.5, 0, 5.0, relative=False)
        hlc_drone5.go_to(-0.4,0.6928203230,0.5, 0, 5.0, relative=False)
        hlc_drone6.go_to(-0.4,-0.6928203230,0.5, 0, 5.0, relative=False)
        time.sleep(5)

        # 각 드론의 trajectory 실행 + LED 토글
        with concurrent.futures.ThreadPoolExecutor(max_workers=6) as executor:
            futures = []
            futures.append(executor.submit(run_sequence, drone1, drone1_trajectory_id, drone1_duration))
            futures.append(executor.submit(run_sequence, drone2, drone2_trajectory_id, drone2_duration))
            futures.append(executor.submit(run_sequence, drone3, drone3_trajectory_id, drone3_duration))
            futures.append(executor.submit(run_sequence, drone4, drone4_trajectory_id, drone4_duration))
            futures.append(executor.submit(run_sequence, drone5, drone5_trajectory_id, drone5_duration))
            futures.append(executor.submit(run_sequence, drone6, drone6_trajectory_id, drone6_duration))

            concurrent.futures.wait(futures)

        print("All drones finished.")
