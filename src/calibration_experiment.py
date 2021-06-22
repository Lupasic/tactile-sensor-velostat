#!/usr/bin/python3

from  velostat_sensor import VelostatSensor
import time

def velostat_calibration(exp_time=15):
    masses = [0, 0.06, 0.12,0.11,0.16,0.210,0.232,0.264, 0.290,0.325,0.370, 0.432, 0.477, 0.530, 0.590, 0, 0.652, 0.867,
    0.926,0.974, 1.032,1.157]
    message = ["0",
    "glue",
    "glue + rope",
    "metal crap",
    "crap + glue",
    "battery grey",
    "battery gray + printed",
    "battery black",
    "battery blue",
    "brick + glue",
    "brick + crap",
    "brick + crap + glue",
    "brick + gray battery",
    "brick + brick",
    "brick + brick + glue",
    "0",
    "brick + brick + glue + rope",
    "battery pack",
    "battery pack + glue",
    "battery pack + crap",
    "battery pack + crap + glue",
    "battery pack + blue battery"] 
    fl = 0
    i = 0
    while i < len(message):
        a = VelostatSensor(file_name=str(i),folder_name="velostat_data_for_calibration")
        print("Put on sensor "+ message[i])
        input()
        t0= time.time()
        while time.time() - t0 <= exp_time:
            k = a.read_all_data(write_to_file=1,msg=str(masses[i]))
        a.close()
        print("Do you want to continue an experiment, if yes - enter, no - write smth")
        temp = input()
        if temp=="":
            i = i + 1
        if len(temp) > 1:
            print("repeat an exeriment")
                

if __name__ == '__main__':
    velostat_calibration()
    # items = {
    #   0: "0",
    #   0.06: "glue",
    #   0.12: "glue + rope",
    #   0.11: "metal crap",
    #   0.16: "crap + glue",
    #   0.210: "battery grey",
    #   0.232: "battery gray + printed",
    #   0.264: "battery black"}
    # for k, v in items.items():
    #         print(f"key: {k}, value: {v}")
    
    # ttuples = ["0", "0.264", "", "0.06", "0.12 + 0.11", "0", "0.210 + 0.06"]

    # for cur_exp in ttuples:
    #     vv = "Put on cur sensor: "
    #     try:
    #         array_of_indidiv = cur_exp.split(sep="+")
    #         messag = sum(float(i) for i in array_of_indidiv)
    #         for d in array_of_indidiv:
    #             vv = vv + items.get(float(d)) + " + "
    #         wtput = vv
    #     except Exception as e:
    #         print(cur_exp + "is not exist or corrapted \n")
    #         continue
    #     print(messag)
    #     print(wtput)
    #     print("  ")
