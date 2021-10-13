# import urx
from numpy.core.fromnumeric import size
from numpy.lib.function_base import select
from futek_sensor import FutekSensor
import numpy as np
from time import perf_counter, time, sleep
from numpy import array, frexp, sin, cos, pi, linspace, random, matrix
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from multiprocessing import Manager, Process, Value, Array
import rtde_control
import rtde_receive

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf

# https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#speedj-example

# Read more about managers

class UR5e_force:
    def __init__(self, address="172.31.1.25", enable_force = 1, file_name=None, folder_name="futek_data"):
        self._enable_force = enable_force
        self.rob_c = rtde_control.RTDEControlInterface(address)

        p = Process(target=self.__read_data_from_manip,args=(address,))
        p.start()

        # shared memory
        self.Xd_shared = Array('d',[0,0,0])
        self.dXd_shared = Array('d',[0,0,0])
        self.__force_from_sensor = Value('d',0.0)

        self.term_process = Value('i',0)

        # shared memory
        self.Xg_shared = Array('d', [0, 0, 0])
        self.dXg_shared = Array('d', [0, 0, 0])
        self.Fd_ideal_shared = Value('d', 0.0)
        self.Fd_real_shared = Value('d', 0.0)

        self.start_vel_control = Value('i', 0)
        

        # vel and acc constraints
        self.__max_operational_acc = 0.4
        self.__max_lin_acc = 0.5
        self.__max_lin_vel = 0.5

        # sensor parameters
        self.force_threshold = 1

        # todo
        self.COMMON_ORIENT=[3.14,0.1,0]
        self.STARTING_POS = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]

        if self._enable_force:
            self.__start_futek_sensor(file_name,folder_name)

    def __read_data_from_manip(self,address):
        self.rob_r = rtde_receive.RTDEReceiveInterface(address)
        while True:
            self.X_cur_shared[:] =  self.rob_r.getActualTCPPose()
            self.dX_cur_shared[:] =  self.rob_r.getActualTCPSpeed()
            # print(f'{self.X_cur_shared[:]}',flush=True,end='\r')

    def __start_futek_sensor(self, file_name,folder_name):
        self.futek = FutekSensor(debug=0,file_name=file_name, folder_name=folder_name)
        p = Process(target=self.__read_data_from_force_sensor)
        p.start()

    def __read_data_from_force_sensor(self):
        while True:
            rec_val =  self.futek.readData(write_to_file=1)
            if rec_val > self.force_threshold:
                self.__force_from_sensor.value = rec_val
                # print(rec_val)
            else:
                self.__force_from_sensor.value = 0

    def lin(self, target_pos, wait=True):
        if wait:
            asyncc = False
        else:
            asyncc = True    
        self.rob_c.moveL(target_pos, self.__max_lin_vel, self.__max_lin_acc, asyncc)

    def freedrive_transient(self, timeout=20):
        self.rob_c.teachMode()
        sleep(timeout)
        self.rob_c.endTeachMode()

    def up(self, wait=True,dz=0.01):
        if wait:
            asyncc = False
        else:
            asyncc = True  
        cur_pose = self.X_cur_shared[:]
        cur_pose[2] += dz
        # cur_pose[3] = -cur_pose[3]
        # cur_pose[4] = -cur_pose[4]
        if self._debug:
            print(cur_pose)
        self.rob_c.moveL(cur_pose, self.__max_lin_vel, self.__max_lin_acc, asyncc)

    def basic_start(self, updz = 0.008):
        starting_pos = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]
        print("start freedrive")
        self.freedrive_transient(5)
        print("end freedrive")
        sensor_init_pose = self.X_cur_shared[:]
        print(sensor_init_pose)
        sensor_init_pose = sensor_init_pose[:3] + self.COMMON_ORIENT
        sensor_init_pose[2]=sensor_init_pose[2]+updz
        print(sensor_init_pose)
        self.lin(starting_pos)
        return sensor_init_pose, starting_pos

    def check_finish_motion(self,time =0):
        eps = 1e-3
        a = np.linalg.norm(array(self.Xd_shared[:])-array(self.X_cur_shared[:3])) <= eps
        b = np.linalg.norm(array(self.dXd_shared[:]) - array(self.dX_cur_shared[:3])) <= eps
        c = abs(self.Fd_real_shared.value - self.__force_from_sensor.value) <= eps
        if  a and b and c:
            return True
        else:
            return False

    def point_load(self, sensor_pos, init_height=0.2, needed_force=0):
        self.start_vel_control.value = 0
        starting_pos = sensor_pos.copy()
        starting_pos[2] = init_height
        self.lin(starting_pos)
        self.define_new_state(sensor_pos[:3],vel_g=[0,0,0],force_g_i=needed_force,force_g_r=0)
        self.start_vel_control.value = 1
        while not self.check_finish_motion():
            continue
        self.start_vel_control.value = 0
        self.define_new_state(starting_pos[:3],vel_g=[0,0,0],force_g_i=0,force_g_r=0)
        self.start_vel_control.value = 1
        sleep(0.1)
        while not ur_robot.check_finish_motion():
            continue

if __name__ == '__main__':
    pass