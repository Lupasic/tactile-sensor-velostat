#!/usr/bin/python3

from  velostat_sensor import VelostatSensor
from futek_sensor import FutekSensor
import time

from dataclasses import dataclass

@dataclass
class TimeLimit:
    start: float
    stop: float


sensor_names = ("old_1","old_2","new_1","new_3_triple")

def question_1():
    #  Correlation between mass and velostat data
    experiments = {"e_sm":"empty->0.064(glue)",
                    "e_md":"empty->0.325(xx)",
                    "e_bg":"empty->0.652(xx)",
                   "m_p_sm":"mass->+0.060(glue)",
                   "m_p_md":"mass->+0.150(xx)",
                   "m_p_bg":"mass->+0.300(xx)"}
    attempts = 3
    # time limit in seconds
    time_limit = 120

    for cur_sensor in sensor_names:
        print("Make the experiment with " + cur_sensor + "sensor. \n Press enter if ready")
        input()
        for cur_exp_key, cur_exp_val in experiments.items():
            print("Your experiment is " + cur_exp_key + ". \n Press enter if ready")
            input()
            for cur_attempt in range(attempts):
                print("Now is a " + cur_attempt + "Attempt. \n Press enter if ready")
                input()
                fl = False
                while fl == False:
                    a = VelostatSensor(file_name=cur_exp_key+"_"+str(cur_attempt),folder_name="question_1/"+cur_sensor+"/"+ cur_exp_key +"/attempt_"+str(cur_attempt))
                    print("Put on sensor "+ cur_exp_val)
                    input()
                    t0= time.time()
                    while time.time() - t0 <= time_limit:
                        k = a.read_all_data(write_to_file=1,msg=cur_exp_key)
                    a.close()
                    print("Do you want to continue an experiment, if yes - enter, no - write smth")
                    temp = input()
                    if temp=="":
                        fl = True
                    if len(temp) > 1:
                        print("repeat an exeriment")
    

def question_2():
    pass
    # TODO

def question_3():
    experiments = {0:"0",
                    0.06: "glue",
                    0.12: "glue + rope",
                    0.11: "metal crap",
                    0.16: "crap + glue",
                    0.21: "battery grey",
                    0.232: "battery grey + printed",
                    0.264: "battery black",
                    0.29: "battery blue",
                    0.325: "brick + glue",
                    0.370: "brick + crap",
                    0.432 :"brick + crap + glue",
                    0.477 :"brick + gray battery",
                    0.530 :"brick + brick",
                    0.590 :"brick + brick + glue",
                    0.652 :"brick + brick + glue + rope"}
    time_limits = [TimeLimit(0,3),TimeLimit(10,10)]
    attempts = 3
    for time_limit in time_limits:
        print("On these experiments you should put after start on " + time_limit + "second. \n Press enter if ready")
        input()
        for cur_sensor in sensor_names:
            print("Make the experiment with " + cur_sensor + "sensor. \n Press enter if ready")
            input()
            for cur_exp_key, cur_exp_val in experiments.items():
                print("Your experiment is to put this" + cur_exp_key + " on sensor. \n Press enter if ready")
                input()
                for cur_attempt in range(attempts):
                    print("Now is a " + cur_attempt + "Attempt. \n Press enter if ready")
                    input()
                    fl = False
                    while fl == False:
                        a = VelostatSensor(file_name=cur_exp_key+"_"+str(cur_attempt),
                        folder_name="question_3_"+time_limit.stop+"s/"+cur_sensor+"/mass_"+ str(cur_exp_key) +"/attempt_"+str(cur_attempt))
                        print("Put on sensor "+ cur_exp_val)
                        input()
                        time.sleep(time_limit.start)
                        t0= time.time()
                        while time.time() - t0 <= time_limit.stop:
                            k = a.read_all_data(write_to_file=1,msg=str(cur_exp_key))
                        a.close()
                        print("Do you want to continue an experiment, if yes - enter, no - write smth")
                        temp = input()
                        if temp=="":
                            fl = True
                        if len(temp) > 1:
                            print("repeat an exeriment")



def question_4():
    pass

def question_5():
    attempts = 3
    # time limit in seconds
    time_limit = TimeLimit(12,10)

    for cur_sensor in sensor_names + ["futek"]:
        print("Make the experiment with " + cur_sensor + "sensor. \n Press enter if ready")
        input()
        for cur_attempt in range(attempts):
            print("Now is a " + cur_attempt + "Attempt. \n Press enter if ready")
            input()
            fl = False
            while fl == False:
                if cur_sensor is not "futek":
                    a = VelostatSensor(file_name="autocorr"+str(cur_attempt),
                    folder_name="question_5/"+cur_sensor+"/autocorr/attempt_"+str(cur_attempt))
                else:
                    a = FutekSensor(file_name="autocorr"+str(cur_attempt),
                    folder_name="question_5/"+cur_sensor+"/autocorr/attempt_"+str(cur_attempt))
                print("Put on sensor 2 bricks, enter, wait while data starts to flow, put off bricks, wait till end")
                input()
                time.sleep(time_limit.start)
                t0= time.time()
                while time.time() - t0 <= time_limit.stop:
                    k = a.read_all_data(write_to_file=1,msg="autocorr")
                a.close()
                print("Do you want to continue an experiment, if yes - enter, no - write smth")
                temp = input()
                if temp=="":
                    fl = True
                if len(temp) > 1:
                    print("repeat an exeriment")


if __name__ == '__main__':
    question_1()
    question_3()
    question_5()
    question_2()
    question_4()