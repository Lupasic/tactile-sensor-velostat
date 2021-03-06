#!/usr/bin/python3

from  velostat_sensor import VelostatSensor
from futek_sensor import FutekSensor
from ur10e import UR10
import ur10e
import datetime
import time
import multiprocessing

from dataclasses import dataclass

@dataclass
class TimeLimit:
    start: float
    stop: float

sensor_names = ["old_1","old_2","new_1","new_3_triple", "double"]
# sensor_names = ["new_1","new_3_triple", "double"]

q = multiprocessing.Queue()

def read_vel_data(velostat,pipe_to_sensor_r = None, pipe_to_main_s = None):
    while True:
        k = velostat.readData(write_to_file=1)
        if pipe_to_sensor_r != None and pipe_to_sensor_r.poll():
            pipe_to_main_s.send(k)
            pipe_to_sensor_r.recv()

def read_futek_data(futek,pipe_to_sensor_r = None, pipe_to_main_s = None):
    while True:
        k = futek.readData(write_to_file=1)
        if pipe_to_sensor_r != None and pipe_to_sensor_r.poll():
            pipe_to_main_s.send(k)
            pipe_to_sensor_r.recv()


def question_1():
    #  Correlation between mass and velostat data
    experiments = {"e_sm":"empty->0.050(red button)",
                    "e_md":"empty->0.267(brick)",
                    "e_bg":"empty->0.64(2 bricks + crap)",
                   "m_p_sm":"mass (brick)->+0.050(red button)",
                   "m_p_md":"mass->+0.150(crap medium + button)",
                   "m_p_bg":"mass->+0.268(brick)"}
    attempts = 3
    # time limit in seconds
    time_limit = 120

    for cur_sensor in sensor_names:
        print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
        input()
        for cur_exp_key, cur_exp_val in experiments.items():
            print("Your experiment is " + str(cur_exp_val) + ". \n Press enter if ready")
            input()
            for cur_attempt in range(attempts):
                print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                input()
                fl = False
                while fl == False:
                    a = VelostatSensor(file_name=str(cur_exp_key)+"_"+str(cur_attempt),folder_name="question_1/"+cur_sensor+"/"+ cur_exp_key +"/attempt_"+str(cur_attempt))
                    t0= time.time()
                    while time.time() - t0 <= time_limit:
                        k = a.readData(write_to_file=1,msg=cur_exp_key)
                    a.close()
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")
    

def test_sensor_on_multiroll(ur10, sensor_init_pose, starting_pos, cur_force, touch_ground_vel):
    ur10.test_flat_sensor_rolling_load(sensor_init_pose, 30,14,0,1,10,needed_force=ur10.futek.F2raw(cur_force), touch_ground_vel=touch_ground_vel)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.futek.close()

def question_2():
    experiments = {0.01:"ground speed 0.01",
                    0.05:"ground speed 0.05)",
                    0.1:"ground speed 0.1"}
    attempts = 3
    for cur_sensor in sensor_names:
        print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
        print("took the rolling end-effector and after enter navigate manipulator in top left corner of sensor")
        input()
        fl_base = False
        while fl_base == False:
            while True:
                try:
                    temp_ur10 = UR10(enable_force=0)
                    break
                except Exception:
                    print("Plug out ethernet cable and plug in again")
            sensor_init_pose, starting_pos = ur10e.basic_start(temp_ur10,updz=0.006)
            temp_ur10 = None
            print("Are you sure that base is correct?, if yes - enter, no - write smth")
            temp = input()
            if temp=="":
                fl_base = True
            if len(temp) > 1:
                print("repeat base instalation")
        for cur_exp_key, cur_exp_val in experiments.items():
            print("Your experiment is install " + cur_exp_val + ". \n Press enter if ready")
            input()
            for cur_attempt in range(attempts):
                print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                input()
                fl = False
                while fl == False:
                    extra_futek = FutekSensor(port='/dev/ttyUSB2',file_name="futek_extra_"+ str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_2/"+cur_sensor+"/speed_"+ str(cur_exp_key) +"/attempt_"+str(cur_attempt))
                    cur_vel = VelostatSensor(file_name=str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_2/"+cur_sensor+"/speed_"+ str(cur_exp_key) +"/attempt_"+str(cur_attempt))
                    p1 = multiprocessing.Process(target=read_futek_data, args=(extra_futek,))
                    p2 = multiprocessing.Process(target=read_vel_data, args=(cur_vel,))
                    while True:
                        try:
                            cur_ur10 = UR10(enable_force=1,
                    file_name="futek_"+ str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_2/"+cur_sensor+"/speed_"+ str(cur_exp_key) +"/attempt_"+str(cur_attempt))
                            print(cur_ur10.rob)
                            break
                        except Exception:
                            print("Plug out ethernet cable and plug in again")
                    p1.start() 
                    p2.start()
                    print("start")
                    test_sensor_on_multiroll(cur_ur10, sensor_init_pose, starting_pos, -1.5, cur_exp_key)
                    p1.terminate()
                    p2.terminate()                 
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")

def question_3():
    experiments = {0:"0",
                    0.035: "crap small",
                    0.05: "button",
                    0.086: "button + crap small",
                    0.11: "crap big",
                    0.144: "crap small + crap big",
                    0.168: "crap small + crap big + motor",
                    0.207: "crap medium + crap big",
                    0.255: "crap medium + crap big + button",
                    0.301: "brick + crap small",
                    0.350: "brick + crap small + button",
                    0.404 :"brick + crap small + crap medium",
                    0.450 :"brick + crap small + crap medium + button",
                    0.530 :"brick + brick",
                    0.580 :"brick + brick + button",
                    0.640 :"brick + brick + crap big",
                    0.674 :"brick + brick + crap small + crap big",
                    0.738 :"brick + brick + crap medium + crap big"}
    time_limits = [TimeLimit(0,3),TimeLimit(10,10)]
    attempts = 5
    for time_limit in time_limits:
        print("On these experiments you should put after start on " + str(time_limit.stop) + "second. \n Press enter if ready")
        input()
        for cur_sensor in sensor_names:
            print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
            input()
            for cur_exp_key, cur_exp_val in experiments.items():
                print("Your experiment is to put " + cur_exp_val + " on sensor. \n Press enter if ready")
                input()
                for cur_attempt in range(attempts):
                    print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                    input()
                    fl = False
                    while fl == False:
                        a = VelostatSensor(file_name=str(cur_exp_key)+"_"+str(cur_attempt),
                        folder_name="question_3_"+str(time_limit.stop)+"s/"+cur_sensor+"/mass_"+ str(cur_exp_key) +"/attempt_"+str(cur_attempt))
                        time.sleep(time_limit.start)
                        t0= time.time()
                        while time.time() - t0 <= time_limit.stop:
                            k = a.readData(write_to_file=1,msg=str(cur_exp_key))
                        a.close()
                        print("Do you want to continue an experiment, if yes - enter, no - write smth")
                        temp = input()
                        if temp=="":
                            fl = True
                        if len(temp) > 1:
                            print("repeat an exeriment")



def make_sensor_load_map(ur10, sensor_init_pose, starting_pos, cur_force, lp, wp, touch_ground_vel=0.005):
    ur10.test_flat_sensor_point_load(sensor_init_pose,14,14,lp,wp,0,1,needed_force=ur10.futek.F2raw(cur_force), touch_ground_vel=touch_ground_vel)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.futek.close()
 


# rolling - 94 gramm 
def question_4():
    experiments = {"3_p":"smallest (3mm) pike 4",
                    "6_p":"small (6mm) pike 5",
                    "9_p":"medium (9mm) pike 6",
                   "12_p":"big (12mm) pike 6",
                   "15_p":"biggest (15mm) pike 9"}
    attempts = 3
    for cur_sensor in sensor_names:
        print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
        print("took the smallest pike and after enter navigate manipulator in top left corner of sensor")
        input()
        fl_base = False
        while fl_base == False:
            while True:
                try:
                    temp_ur10 = UR10(enable_force=0)
                    break
                except Exception:
                    print("Plug out ethernet cable and plug in again")
            sensor_init_pose, starting_pos = ur10e.basic_start(temp_ur10,updz=0.006)
            temp_ur10 = None
            print("Are you sure that base is correct?, if yes - enter, no - write smth")
            temp = input()
            if temp=="":
                fl_base = True
            if len(temp) > 1:
                print("repeat base instalation")
        for cur_exp_key, cur_exp_val in experiments.items():
            print("Your experiment is install " + cur_exp_val + " end-effector. \n Press enter if ready")
            input()
            for cur_attempt in range(attempts):
                print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                input()
                fl = False
                while fl == False:
                    extra_futek = FutekSensor(port='/dev/ttyUSB2',file_name="futek_extra_"+ str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_4/"+cur_sensor+"/"+ cur_exp_key +"/attempt_"+str(cur_attempt))
                    cur_vel = VelostatSensor(file_name=str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_4/"+cur_sensor+"/"+ cur_exp_key +"/attempt_"+str(cur_attempt))
                    p1 = multiprocessing.Process(target=read_futek_data, args=(extra_futek,))
                    p2 = multiprocessing.Process(target=read_vel_data, args=(cur_vel,))
                    while True:
                        try:
                            cur_ur10 = UR10(enable_force=1,
                    file_name="futek_"+ str(cur_exp_key)+"_"+str(cur_attempt),
                    folder_name="question_4/"+cur_sensor+"/"+ cur_exp_key +"/attempt_"+str(cur_attempt))
                            print(cur_ur10.rob)
                            break
                        except Exception:
                            print("Plug out ethernet cable and plug in again")
                    p1.start() 
                    p2.start()
                    print("start")
                    make_sensor_load_map(cur_ur10, sensor_init_pose, starting_pos, 3, 4, 4)
                    p1.terminate()
                    p2.terminate()                   
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")

def question_5():
    attempts = 3
    # time limit in seconds
    time_limits = [TimeLimit(10,15), TimeLimit(30,15), TimeLimit(60,15)]

    for cur_sensor in sensor_names + ["futek"]:
        print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
        input()
        for time_limit in time_limits:
            print ("Time for waiting is " + str(time_limit.start))
            input()
            for cur_attempt in range(attempts):
                print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                input()
                fl = False
                while fl == False:
                    if cur_sensor != "futek":
                        a = VelostatSensor(file_name="exp_1_5"+str(cur_attempt),
                        folder_name="question_1_5/"+cur_sensor+"/autocorr_" + str(time_limit.start) + "/attempt_"+str(cur_attempt))
                    else:
                        a = FutekSensor(file_name="autocorr"+str(cur_attempt),
                        folder_name="question_1_5/"+cur_sensor+"/autocorr_" + str(time_limit.start) + "/attempt_"+str(cur_attempt))
                    print("Put on sensor 2 bricks, enter, wait while data starts to flow, put off bricks, wait till end")
                    input()
                    time.sleep(time_limit.start)
                    t0= time.time()
                    while time.time() - t0 <= time_limit.stop:
                        # WONT work with FUTEK!! need to fix function
                        k = a.readData(write_to_file=1,msg="autocorr")
                    a.close()
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")


def question_1_5():
    experiments = {"e_sm":"empty->0.050(red button)",
                    "e_md":"empty->0.267(brick)",
                    "e_bg":"empty->0.64(2 bricks + crap)",
                   "m_p_sm":"mass (brick)->+0.050(red button)",
                   "m_p_md":"mass->+0.150(crap medium + button)",
                   "m_p_bg":"mass->+0.268(brick)"}
    attempts = 3
    # time limit in seconds
    time_limits = [TimeLimit (60,20)]
    for cur_sensor in sensor_names + ["futek"]:
            print("Make the experiment with " + cur_sensor + " sensor. \n Press enter if ready")
            input()
            for cur_exp_key, cur_exp_val in experiments.items():
                print("Your experiment is " + str(cur_exp_val) + ". \n Press enter if ready")
                input()
                for time_limit in time_limits:
                    print ("Time for waiting is " + str(time_limit.start))
                    input()
                    for cur_attempt in range(attempts):
                        print("Now is a " + str(cur_attempt) + " attempt. \n Press enter if ready")
                        input()
                        fl = False
                        while fl == False:
                            # The way how i am using pipe here
                            # https://stackoverflow.com/questions/67609301/how-can-i-get-the-most-recent-value-from-a-data-stream-in-a-separate-python-mult
                            # https://docs-python.ru/standart-library/paket-multiprocessing-python/klass-pipe-modulja-multiprocessing/
                            pipe_to_main_r, pipe_to_main_s = multiprocessing.Pipe(duplex=False)
                            pipe_to_sensor_r, pipe_to_sensor_s = multiprocessing.Pipe(duplex=False)
                            # TODO naming
                            fname = str(cur_exp_key)+"_"+str(cur_attempt)
                            folname = "question_1_5/"+cur_sensor+"/"+ str(cur_exp_key) + "/" + str(time_limit.start) + "_" + str(time_limit.stop) + "/attempt_"+str(cur_attempt)
                            a = []
                            if cur_sensor != "futek":
                                for cur_name in ["stage_1_", "stage_2_", "stage_3_", "whole_"]:
                                    a.append(VelostatSensor(file_name= cur_name + fname, folder_name=folname))
                                p1 = multiprocessing.Process(target=read_vel_data, args=(a[-1],pipe_to_sensor_r,pipe_to_main_s,))
                            else:
                                for cur_name in ["stage_1_", "stage_2_", "stage_3_", "whole_"]:
                                    a.append(FutekSensor(file_name= cur_name + fname, folder_name=folname))
                                p1 = multiprocessing.Process(target=read_futek_data, args=(a[-1],pipe_to_sensor_r,pipe_to_main_s,))
                            print("Put " + str(cur_exp_val) + "(first part) and immedietely press enter")
                            input()
                            pipe_to_sensor_s.send(None)
                            p1.start()
                            t0= time.time()
                            while time.time() - t0 <= 10:
                                string_data = str(pipe_to_main_r.recv()[0]) + " "
                                print(string_data)
                                a[0].text_file.write(string_data + str(datetime.datetime.now().timestamp()) + "\n")
                                pipe_to_sensor_s.send(None)
                            a[0].close()
                            print("Put " + str(cur_exp_val) + "(second part) and immedietely push enter")
                            input()
                            t0= time.time()
                            # clear last data from pipe
                            pipe_to_main_r.recv()[0]
                            pipe_to_sensor_s.send(None)
                            while time.time() - t0 <= time_limit.start:
                                string_data = str(pipe_to_main_r.recv()[0]) + " "
                                print(string_data)
                                a[1].text_file.write(string_data + str(datetime.datetime.now().timestamp()) + "\n")
                                pipe_to_sensor_s.send(None)
                            a[1].close()
                            print("Remove everything after pushing enter, not before)")
                            input()
                            t0= time.time()
                            # clear last data from pipe
                            pipe_to_main_r.recv()[0]
                            pipe_to_sensor_s.send(None)

                            while time.time() - t0 <= time_limit.stop:
                                string_data = str(pipe_to_main_r.recv()[0]) + " "
                                print(string_data)
                                a[2].text_file.write(string_data + str(datetime.datetime.now().timestamp()) + "\n")
                                pipe_to_sensor_s.send(None)
                            a[2].close()
                            p1.terminate()                   
                            print("Do you want to continue an experiment, if yes - enter, no - write smth")
                            temp = input()
                            if temp=="":
                                fl = True
                            if len(temp) > 1:
                                print("repeat an exeriment")


if __name__ == '__main__':
    # question_1()
    # question_3()
    question_1_5()
    # question_2()
    # question_4()