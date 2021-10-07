from multiprocessing import Process, Value, Array
import random
from time import perf_counter, sleep


def funk1(Xd_shared, dXd_shared):
    t0 = perf_counter()
    t1 = 0

    while True:
        t = perf_counter() -t0
        if t - t1 > 5:
            # dXd_shared[:] = Xd_shared[:]
            # dXd_shared[:] = [1, 0, force.value]
            # print(f' funk1 {dXd_shared[:]} {force.value}',)
            t1 = t


def funk2(dXd_shared, force):
    t0 = perf_counter()
    t1 = 0
    dXd_shared[:] = [1, 0, force.value]
    while True:
        t = perf_counter() -t0
        if t - t1 > 5:
            # dXd_shared[:] = [1, 0, force.value]
            print(f' funk2 {dXd_shared[:]} {force.value}',)
            t1 = t


def funk3(force, dXd_shared):
        dx = [1,1,1]
        print(dx)
        while True:
            force.value = random.uniform(0,5)
            print(dXd_shared[:])
            if dx == dXd_shared[:]:
                print("I am the boss of this gym")
            sleep(5)  



if __name__ == '__main__':
    Xd_shared = Array('d',[0,0,0])
    dXd_shared = Array('d',[0,0,0])
    force = Value('d',0.0)
    p1 = Process(target=funk1, args=(Xd_shared, dXd_shared))
    p2 = Process(target=funk2, args=(dXd_shared, force))
    p3 = Process(target=funk3, args=(force,dXd_shared))
    p2.start()
    p1.start()
    p3.start()
    t0 = perf_counter()
    t1 = 0
    while True:
        t = perf_counter() -t0
        # if t - t1 < 2:
        #     # force.value = 10
        #     print(f' main1 {dXd_shared[:]} {force.value}',)
        if t - t1 > 10:
            # force.value = 10
            # dXd_shared[:] = [1, 2, [force.value]]
            dXd_shared[:] = [1,1,1]
            print(f' main21 {dXd_shared[:]} {force.value}',)
            print(f' main22 {dXd_shared[:]} {force.value}',)
            t1 = t
