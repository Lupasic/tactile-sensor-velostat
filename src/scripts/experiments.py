from multiprocessing import Process
from time import sleep
from ur5e import UR5e
from velostat_sensor import VelostatSensor

def read_data_from_velostat(velostat):
    while True:
        velostat.readData(write_to_file=1)


def sensor_multitouch():
    robot = UR5e(enable_force=1)
    velostat = VelostatSensor(debug=True)
    process_velostat = Process(target=read_data_from_velostat,args=(velostat,))
    process_velostat.start()
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        robot.sensor_point_load(sensor_pos,15,15,lp=3,wp=3,repeats=1,Fd_ideal=200,Fd_real=90)
        robot.shutdown_robot()
        print("I am here")
        process_velostat.terminate()
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

if __name__ == '__main__':
    sensor_multitouch()




