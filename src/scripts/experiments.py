from multiprocessing import Process
from time import sleep, perf_counter
from ur5e import UR5e
from velostat_sensor import VelostatSensor
import datetime

def read_data_from_velostat(velostat):
    while True:
        velostat.readData(write_to_file=1)

def starting_experiment_time(exp_name):
    experiment_start_timestamp = datetime.datetime.now().timestamp()
    print(f'Experiment starts {experiment_start_timestamp}')
    text_file_name = "/home/app/tactile_sensor_velostat/experimental_data/starting_exp_time/starting_experiment_" + exp_name +".txt"
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
    # sensor 1
    # sensor_pos = [-0.5792462414039549, 0.22161784199712997, 0.06818361500999862, 3.14, 0.1, 0]
    # sensor 2
    # sensor_pos = [-0.5809926689165119, 0.19787852826315594, 0.06846670629401433, 3.14, 0.1, 0]
    # sensor 3
    sensor_pos = [-0.5817922224967008, 0.17354650244643094, 0.06905110207521711, 3.14, 0.1, 0]
    exp_names = ["pike5_sensor3_exp1","pike5_sensor3_exp2"]
    folder_exp_name = "pike5"
    robot = UR5e(enable_force=1,file_name=exp_names[0],folder_name="futek_data/"+folder_exp_name)
    fll = 0
    for exp_name in exp_names:
        if fll == 1:
            robot.set_force_sensor(exp_name,folder_name="futek_data/"+folder_exp_name)
        fll = 1
        velostat = VelostatSensor(debug=False,file_name=exp_name,folder_name="velostat_data/"+folder_exp_name)
        process_velostat = Process(target=read_data_from_velostat,args=(velostat,))
        process_velostat.start()
        try:
            robot.run_env_for_lin_force()
            sleep(1)
            if fl==0:
                sensor_pos = robot.basic_start()
                fl = 1
            starting_experiment_time(exp_name)
            robot.sensor_point_load(sensor_pos,15,15,lp=5,wp=5,repeats=1,Fd_ideal=43,Fd_real=49,h_init=0.075)
            
            print("I am here")
            # kill velostat and futek for new iteration
            sleep(0.5)
            robot.disable_force_sensor()
            process_velostat.terminate()
            sleep(1)
        except KeyboardInterrupt:
            robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
            print('Robot is stopped')
    robot.shutdown_robot()


def data_for_least_square():
        experiments = {0.020:"empty->3d printed stuff",
                    0.1:"empty->black metal ",
                    0.252:"empty->phone"}
        time_limit = 60
        VELOSTAT_THRESHOLD = 1300
        sensor_names = ["sensor1", "sensor2"]
        for cur_sensor in sensor_names:
            print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
            input()
            for cur_exp_key, cur_exp_val in experiments.items():
                print("Your experiment is " + str(cur_exp_val) + ". \n Press enter if ready")
                input()
                fl = False

                while fl == False:
                    velostat = VelostatSensor(file_name="static_weigt"+cur_sensor + "_"+str(cur_exp_key),debug=1)
                    K = True
                    fl1 = False
                    write_to_file_var=0                    
                    t0= perf_counter()
                    while True and K:
                        data = velostat.readData(write_to_file=write_to_file_var,msg=str(cur_exp_key))[0]
                        if data < VELOSTAT_THRESHOLD:
                            if fl1 == False:
                                write_to_file_var = 1
                                fl1 = True
                            K  = perf_counter() - t0 <= time_limit
                    velostat.close()
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")

if __name__ == '__main__':
    # sensor_multitouch("pike0_sensor1_exp1")
    multiple_exp_sensor_multitouch(fl=1)
    # data_for_least_square()



