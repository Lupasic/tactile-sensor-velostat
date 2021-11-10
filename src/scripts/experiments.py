from multiprocessing import Process
from time import sleep
from ur5e import UR5e
from velostat_sensor import VelostatSensor
import datetime

def read_data_from_velostat(velostat):
    while True:
        velostat.readData(write_to_file=1)

def starting_experiment_time():
    experiment_start_timestamp = datetime.datetime.now().timestamp()
    print(f'Experiment starts {experiment_start_timestamp}')
    text_file = open("/home/app/tactile_sensor_velostat/experimental_data/starting_experiment.txt","w")
    text_file.write(str(experiment_start_timestamp) + "\n")
    text_file.close()


def sensor_multitouch():
    robot = UR5e(enable_force=1,file_name="smallest_pike")
    velostat = VelostatSensor(debug=False,file_name="smallest_pike")
    process_velostat = Process(target=read_data_from_velostat,args=(velostat,))
    process_velostat.start()
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        starting_experiment_time()
        robot.sensor_point_load(sensor_pos,15,15,lp=3,wp=3,repeats=1,Fd_ideal=28,Fd_real=28)
        robot.shutdown_robot()
        print("I am here")
        process_velostat.terminate()
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

if __name__ == '__main__':
    sensor_multitouch()




