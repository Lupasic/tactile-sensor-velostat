import multiprocessing 
import os
import time

from dataclasses import dataclass

@dataclass
class TimeLimit:
    start: float
    stop: float

def worker1(time_limit,q): 
    print("ID of worker1: {}".format(os.getpid()))
    print(time_limit.stop)
    # child_conn.send("1")
    # child_conn.send("2")
    # child_conn.send("3")
    q.put_nowait("1")
    q.put_nowait("2")
    q.put("3")
    time.sleep(10)
    q.put_nowait("5")
    # child_conn.send("5")

def worker2():
    while True:
        print("ID of worker2: {}".format(os.getpid()))
        time.sleep(2)

if __name__ == "__main__":
    parent_conn, child_conn = multiprocessing.Pipe()
    q = multiprocessing.Queue()
    parent_conn.poll()
    print("ID of main process: {}".format(os.getpid())) 
    p1 = multiprocessing.Process(target=worker1,args=(TimeLimit(10,20), q,)) 
    p2 = multiprocessing.Process(target=worker2) 
    p1.start() 
    p2.start()
    print("ID of process p1: {}".format(p1.pid))
    print("dsd " + str(q.qsize()))
    # print(q.get_nowait())
    print("dsd " + str(q.qsize()))
    print("ID of process p2: {}".format(p2.pid))  
    print("Process p1 is alive: {}".format(p1.is_alive())) 
    p1.join() 
    print("processes finished execution!") 
    print("Process p2 is alive: {}".format(p2.is_alive()))
    p2.terminate()
    print("dsd " + str(q.qsize()))
    print(q.get_nowait())
    print("dsd " + str(q.qsize()))
    # print(parent_conn.recv()[-1])
    # print(parent_conn.recv())
    # print(parent_conn.recv())
    print("meow")