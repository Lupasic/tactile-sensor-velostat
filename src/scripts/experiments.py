from multiprocessing import Process
from time import sleep
from ur5e import UR5e
from velostat_sensor import VelostatSensor
import datetime

def read_data_from_velostat(velostat):
    while True:
        velostat.readData(write_to_file=1)

def starting_experiment_time(exp_name):
    experiment_start_timestamp = datetime.datetime.now().timestamp()
    print(f'Experiment starts {experiment_start_timestamp}')
    text_file_name = "/home/app/tactile_sensor_velostat/experimental_data/starting_experiment_" + exp_name +".txt"
    text_file = open(text_file_name,"w")
    text_file.write(str(experiment_start_timestamp) + "\n")
    text_file.close()


def sensor_multitouch(exp_name):
    robot = UR5e(enable_force=1,file_name=exp_name)
    velostat = VelostatSensor(debug=False,file_name=exp_name)
    process_velostat = Process(target=read_data_from_velostat,args=(velostat,))
    process_velostat.start()
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        starting_experiment_time(exp_name)
        robot.sensor_point_load(sensor_pos,13,15,lp=4,wp=4,repeats=1,Fd_ideal=10,Fd_real=9.1,h_init=0.075)
        robot.shutdown_robot()
        print("I am here")
        process_velostat.terminate()
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

def multiple_exp_sensor_multitouch(fl=0):
    sensor_pos = [-0.7021411229811721, 0.21766862016381397, 0.06957241936962599, 3.14, 0.1, 0]
    for exp_name in ["pike4_sensor1_exp3"]:
        robot = UR5e(enable_force=1,file_name=exp_name)
        velostat = VelostatSensor(debug=False,file_name=exp_name)
        process_velostat = Process(target=read_data_from_velostat,args=(velostat,))
        process_velostat.start()
        try:
            robot.run_env_for_lin_force()
            sleep(1)
            if fl==0:
                sensor_pos = robot.basic_start()
                fl = 1
            starting_experiment_time(exp_name)
            robot.sensor_point_load(sensor_pos,13,15,lp=4,wp=4,repeats=1,Fd_ideal=42,Fd_real=49,h_init=0.075)
            robot.shutdown_robot()
            print("I am here")
            process_velostat.terminate()
            sleep(1)
        except KeyboardInterrupt:
            robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
            print('Robot is stopped')

if __name__ == '__main__':
    # sensor_multitouch("pike0_sensor1_exp1")
    multiple_exp_sensor_multitouch(fl=1)




