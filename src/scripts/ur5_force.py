import urx
from math import pi
from futek_sensor import FutekSensor
import numpy as np
from time import perf_counter
from numpy import array, frexp, sin, cos, pi, linspace, random, matrix
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from multiprocessing import Process, Value

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf


class UR5e_force:
    def __init__(self, address="172.31.1.25", debug = 1, enable_force = 1, file_name=None, folder_name="futek_data"):
        self._debug = debug
        self._enable_force = enable_force
        self.rob = urx.Robot(address, use_rt=False)
        if self._enable_force:
            self.futek = FutekSensor(debug=1,file_name=file_name, folder_name=folder_name)
        # vel and acc constraints
        self.__max_operational_acc = 0.1
        self.__max_lin_acc = 0.1
        self.__max_lin_vel = 0.1

        # modification trajectory tune params
        self.__vir_stiff = 500
        self.__freq_mod_traj = 100

        # vel control parameters
        self.__freq_log = 100
        self.__k = 1

        # todo
        self._common_orient=[3.14,0.1,0]

        self.starting_pose = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]


    def lin(self, target_pos, wait=True):
        self.rob.movel(target_pos, acc= self.__max_lin_acc, vel= self.__max_lin_vel ,wait=wait)


    def up(self, wait=True,dz=0.01):
        if wait:
            self.rob.up(z=dz, acc= self.__max_lin_acc, vel= self.__max_lin_vel)
        else:
            cur_pose = self.rob.getl()
            cur_pose[2] += dz
            # cur_pose[3] = -cur_pose[3]
            # cur_pose[4] = -cur_pose[4]
            if self._debug:
                print(cur_pose)
            self.rob.movel(cur_pose, acc= self.__max_lin_acc, vel= self.__max_lin_vel, wait=False, relative=False, threshold=None)


    def __f_x(self, delta_prev,t,F_cur):
        return F_cur - self.__vir_stiff*delta_prev


    def __solve_diff_eq(self, delta_prev, F_cur):
        t0 = perf_counter()
        t = linspace(0,1/self.__freq_mod_traj,3)
        delta = odeint(self.__f_x, delta_prev, t, args = (F_cur,))[-1][0]
        ddelta = F_cur - self.__vir_stiff * delta
        ttt = perf_counter() - t0
        return delta, ddelta, ttt
    
    def __modify_trajectory_based_on_force(self, X_g, dX_g, F_cur):
        # we are interested only in z axis
        z_g = X_g[2]
        dz_g = dX_g[2]
        delta, ddelta, __ = self.__solve_diff_eq(self.__delta_prev, F_cur)
        # in first iteration, __delta_prev_should be defined
        self.__delta_prev = delta
        # modify X_d
        X_d_mod = [X_g[0], X_g[1], z_g + delta]
        dX_d_mod = [dX_g[0], dX_g[1], dz_g + ddelta]
        return X_d_mod, dX_d_mod

    def vel_control(self, X_d, dX_d):
        X_d = array(X_d)
        dX_d = array(dX_d)
        z_act = []
        z_des = []
        time = []
        try: 
            t0 = perf_counter()
            t1 = 0
            while True: 
                t = perf_counter() -t0
                state = self.rob.getl()

                X_cur = array(state[:3])
                U = dX_d + self.__k*(X_d - X_cur)

                if t - t1 >1/self.__freq_log:
            
                    z_act.append(X_cur[2])
                    z_des.append(X_d[2])
                    time.append(t)
                    t1 = t
                    print(f'{z_act[-1].round(3)} {z_des[-1].round(3)} {z_des[-1].round(3) - z_act[-1].round(3)}', end = '\r', flush = True)

                self.rob.speedx("speedl",[U[0],U[1],U[2],0,0,0], self.__max_operational_acc, 5)
                


        except KeyboardInterrupt:
            self.rob.speedx("speedl",[0,0,0,0,0,0], self.__max_operational_acc, 1)
            print('Robot is stopped')

            plt.plot(time, array(z_act), label="actual")
            plt.plot(time, array(z_des), label="needed")
            plt.plot(time, array(z_act) - array(z_des), label="error")
            plt.title("Manipulator control, k = " + str(self.__k))
            plt.xlabel("time, sec")
            plt.ylabel("position, m")
            plt.legend(loc="upper right")
            plt.show()

    def touch_ground_with_constant_force():
        pass

    def roll_with_constant_force():
        pass
    

if __name__ == '__main__':
    ur_robot = UR5e_force(enable_force = 0)
    print(ur_robot)

    X_cur = ur_robot.rob.getl()[:3]
    X_cur[2] = 0.3
    X_d = X_cur

    ur_robot.vel_control(X_d, [0,0,0])
