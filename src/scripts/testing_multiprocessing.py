import multiprocessing 
import os 
import time

from dataclasses import dataclass

@dataclass
class TimeLimit:
    start: float
    stop: float

def worker1(time_limit): 
    print("ID of worker1: {}".format(os.getpid()))
    print(time_limit.stop)
    time.sleep(10)

def worker2():
    while True:
        print("ID of worker2: {}".format(os.getpid()))
        time.sleep(2)

if __name__ == "__main__": 
    print("ID of main process: {}".format(os.getpid())) 
    p1 = multiprocessing.Process(target=worker1,args=(TimeLimit(10,20),)) 
    p2 = multiprocessing.Process(target=worker2) 
    p1.start() 
    p2.start() 
    print("ID of process p1: {}".format(p1.pid)) 
    print("ID of process p2: {}".format(p2.pid))  

    print("Process p1 is alive: {}".format(p1.is_alive())) 
    p1.join() 
    print("processes finished execution!") 
    print("Process p2 is alive: {}".format(p2.is_alive()))
    p2.terminate()
    print("meow")