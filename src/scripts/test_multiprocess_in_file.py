from multiprocessing import Process, Value, Array, Manager
import random
from time import perf_counter, sleep

class Testing:
    def __init__(self) -> None:
        self.Xd_shared = Array('d',[0,0,0])
        self.dXd_shared = Array('d',[0,0,0])
        # self.dXd_shared = [0,0,0]
        self.force = Value('d',0.0)
        mgr = Manager()
        self.namespace = mgr.Namespace()
        self.namespace.my_list = []


    def funk1(self):
        t0 = perf_counter()
        t1 = 0

        while True:
            t = perf_counter() -t0
            if t - t1 > 5:
                # dXd_shared[:] = Xd_shared[:]
                # dXd_shared[:] = [1, 0, force.value]
                # print(f' funk1 {dXd_shared[:]} {force.value}',)
                t1 = t


    def funk2(self):
        t0 = perf_counter()
        t1 = 0
        kek = []
        self.dXd_shared[:] = [1, 0, te.force.value]
        while True:
            t = perf_counter() -t0
            if t - t1 > 5:
                kek.append(1)
                self.namespace.my_list = kek
                # dXd_shared[:] = [1, 0, force.value]
                print(f' funk21 {self.dXd_shared[:]} {self.force.value}',)
                self.dXd_shared[2] = self.force.value
                print(f' funk22 {self.dXd_shared[:]} {self.force.value}',)
                t1 = t


    def funk3(self):
        dx = [1,1,1]
        print(dx)
        while True:
            self.force.value = random.uniform(0,5)
            # print(self.dXd_shared[:])
            # if dx == self.dXd_shared[:]:
                # print("I am the boss of this gym")
            # sleep(5)  



if __name__ == '__main__':
    te = Testing()
    
    p1 = Process(target=te.funk1 )
    p2 = Process(target=te.funk2 )
    p3 = Process(target=te.funk3 )
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
        if t - t1 > 3:
            # force.value = 10
            # te.dXd_shared[:] = [1, 2, te.force.value]
            te.dXd_shared[:] = [1,1,1]
            print(te.Xd_shared[:])
            print(f' main21 {te.dXd_shared[:]} {te.force.value} {te.namespace.my_list}',)
            print(f' main22 {te.dXd_shared[:]} {te.force.value}',)
            t1 = t
