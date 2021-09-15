import urx
from math import pi
from futek_sensor import FutekSensor
import numpy as np
from time import perf_counter
from numpy import array, frexp, sin, cos, pi, linspace, random, matrix
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from multiprocessing import Process, Value, Array

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

        # shared memory
        self.__Xd_shared = Array('d',[0,0,0])
        self.__dXd_shared = Array('d',[0,0,0])
        self.__force_from_sensor = Value('d',0.0)

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

    def __modify_z_g(self, F_d, X_g):
        return X_g[2] - F_d/self.__vir_stiff

    def _force_planner(self, X_g, dX_g, F_cur, F_d):
        X_g[2] = self.__modify_z_g(F_d, X_g)
        try: 
            t0 = perf_counter()
            t1 = 0
            self.__delta_prev = 0
            while True:
                t = perf_counter() -t0 
                if t - t1 >1/self.__freq_mod_traj:
                    X_d_mod, dX_d_mod = self.__modify_trajectory_based_on_force(X_g, dX_g, F_cur)
                    self.__Xd_shared.value = X_d_mod
                    self.__dXd_shared.value = dX_d_mod
                    t1 = t
                    print(f'{X_d_mod} {dX_d_mod}', end = '\r', flush = True)
        except KeyboardInterrupt:
            pass

    def vel_control(self, X_d, dX_d):
        X_d = array(X_d)
        dX_d = array(dX_d)
        dX_cur = None
        x_act = []
        x_des = []
        time = []
        eps = 0.01
        try: 
            t0 = perf_counter()
            t1 = 0
            fl = 0
            while True: 
                t = perf_counter() -t0
                state = self.rob.getl()
                X_cur = array(state[:3])
                U = dX_d + self.__k*(X_d - X_cur)
                self.rob.speedx("speedl",[U[0],U[1],U[2],0,0,0], self.__max_operational_acc, 5)
                
                if t - t1 >1/self.__freq_log:
                    if fl:
                        dX_cur = (X_cur - x_act[-1])/(t-t1)
                    if fl == 0:
                        fl = 1
                    x_act.append(X_cur)
                    x_des.append(X_d)
                    time.append(t)
                    t1 = t
                    print(f'{x_act[-1][2].round(3)} {x_des[-1][2].round(3)} {x_des[-1][2].round(3) - x_act[-1][2].round(3)} {np.linalg.norm((X_d-X_cur))}', end = '\r', flush = True)

                
                
                if dX_cur is not None:
                    if np.linalg.norm((X_d-X_cur)) < eps and np.linalg.norm(dX_d - dX_cur) < eps:
                        # print("Mech")
                        pass
                        # raise(KeyboardInterrupt)
                        # self.rob.speedx("speedl",[0,0,0,0,0,0], self.__max_operational_acc, 1)
                        # raise(KeyboardInterrupt)



        except KeyboardInterrupt:
            self.rob.speedx("speedl",[0,0,0,0,0,0], self.__max_operational_acc, 1)
            print('Robot is stopped')

            plt.plot(time, array(x_act), label=["actual x","actual y","actual z"])
            plt.plot(time, array(x_des), label=["needed x","needed y","needed z"])
            # plt.plot(time, array(x_act) - array(x_des), label="error")
            plt.title("Manipulator control, k = " + str(self.__k))
            plt.xlabel("time, sec")
            plt.ylabel("position, m")
            plt.legend(loc="upper right")
            plt.show()

    def touch_ground_with_constant_force():
        pass

    def roll_with_constant_force():
        pass
    

def read_data_from_force_sensor(futek, received_data):
    while True:
        rec_val =  futek.raw2F(futek.readData(write_to_file=1))
        if rec_val > 1:
            received_data.value = rec_val
        else:
            received_data.value = 0


if __name__ == '__main__':
    ur_robot = UR5e_force(enable_force = 0)
    print(ur_robot)

    X_cur = ur_robot.rob.getl()[:3]
    X_cur[2] = 0.5
    X_cur[1] = 0.2
    X_d = X_cur

    # ur_robot.vel_control(X_d, [0,0,0])
    ur_robot._force_planner(X_d,[0,0,0],0,2)
    print("done")


