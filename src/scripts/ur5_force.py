# import urx
from futek_sensor import FutekSensor
import numpy as np
from time import perf_counter, time, sleep
from numpy import array, frexp, sin, cos, pi, linspace, random, matrix
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from multiprocessing import Process, Value, Array
import rtde_control
import rtde_receive

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf

# https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#speedj-example



class UR5e_force:
    def __init__(self, address="172.31.1.25", debug = 1, enable_force = 1, file_name=None, folder_name="futek_data"):
        self._debug = debug
        self._enable_force = enable_force
        self.rob_c = rtde_control.RTDEControlInterface(address)
        self.rob_r = rtde_receive.RTDEReceiveInterface(address)


        # shared memory
        self.Xd_shared = Array('d',[0,0,0])
        self.dXd_shared = Array('d',[0,0,0])
        self.__force_from_sensor = Value('d',0.0)

        # vel and acc constraints
        self.__max_operational_acc = 0.4
        self.__max_lin_acc = 0.5
        self.__max_lin_vel = 0.5

        # modification trajectory tune params
        self.__vir_stiff = 400000
        self.__freq_mod_traj = 500


        # vel control parameters
        self.__freq_log = 500
        self.__k = 0.4

        # todo
        self._common_orient=[3.14,0.1,0]

        self.starting_pose = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]

        if self._enable_force:
            self.futek = FutekSensor(debug=0,file_name=file_name, folder_name=folder_name)
            p = Process(target=self.__read_data_from_force_sensor)
            p.start()


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
        cur_pose = self.rob_r.getActualTCPPose()
        cur_pose[2] += dz
        # cur_pose[3] = -cur_pose[3]
        # cur_pose[4] = -cur_pose[4]
        if self._debug:
            print(cur_pose)
        self.rob_c.moveL(cur_pose, self.__max_lin_vel, self.__max_lin_acc, asyncc)


    def __f_x(self, delta_prev,t,F_cur):
        return F_cur - self.__vir_stiff*delta_prev


    def __solve_diff_eq(self, delta_prev):
        t0 = perf_counter()
        F_cur = self.__force_from_sensor.value
        t = linspace(0,1/self.__freq_mod_traj,3)
        delta = odeint(self.__f_x, delta_prev, t, args = (F_cur,))[-1][0]
        ddelta = F_cur - self.__vir_stiff * delta
        ttt = perf_counter() - t0
        return delta, ddelta, ttt
    
    def __modify_trajectory_based_on_force(self, X_g, dX_g):
        # we are interested only in z axis
        z_g = X_g[2]
        dz_g = dX_g[2]
        delta, ddelta, __ = self.__solve_diff_eq(self.__delta_prev)
        # in first iteration, __delta_prev_should be defined
        self.__delta_prev = delta
        # modify X_d
        X_d_mod = [X_g[0], X_g[1], z_g + delta]
        dX_d_mod = [dX_g[0], dX_g[1], dz_g + ddelta]
        return X_d_mod, dX_d_mod

    def __modify_z_g(self, F_d, X_g):
        return X_g[2] - F_d/self.__vir_stiff

    def _force_planner(self, X_g, dX_g, F_d):
        X_g[2] = self.__modify_z_g(F_d, X_g)
        try: 
            t0 = perf_counter()
            t1 = 0
            self.__delta_prev = 0
            while True:
                t = perf_counter() -t0 
                if t - t1 >1/self.__freq_mod_traj:
                    X_d_mod, dX_d_mod = self.__modify_trajectory_based_on_force(X_g, dX_g)
                    self.Xd_shared[:] = X_d_mod
                    self.dXd_shared[:] = dX_d_mod
                    t1 = t
                    # print(f'{X_d_mod} {dX_d_mod}', end = '\r', flush = True)
        except KeyboardInterrupt:
            pass

    def vel_control(self, X_d = None, dX_d = None):
        # X_d = array(X_d)
        # dX_d = array(dX_d)
        dX_cur = None
        x_act = []
        x_des = []
        time = []
        force_data = []
        eps = 0.01
        try: 
            t0 = perf_counter()
            t1 = 0
            fl = 0
            i = 0
            while True:
                X_d = array(self.Xd_shared[:])
                
                dX_d = array(self.dXd_shared[:])
                t = perf_counter() -t0
                state = self.rob_r.getActualTCPPose()
                X_cur = array(state[:3])
                U = dX_d + self.__k*(X_d - X_cur)
                # U = self.dXd_shared[:] + self.__k*(self.Xd_shared[:] - X_cur)
                self.rob_c.speedL([U[0],U[1],U[2],0,0,0], self.__max_operational_acc, 1/self.__freq_log)
                if t -t1 < 1/self.__freq_log:
                    sleep(1/self.__freq_log - (t -t1))
                if t - t1 >= 1/self.__freq_log and np.linalg.norm(X_d) > 0.1:
                    x_act.append(list(X_cur))
                    x_des.append(list(X_d))
                    force_data.append(self.__force_from_sensor.value)
                    time.append(t)
                    t1 = t
                    print(f'{i} {x_act[-1][2].round(3)} {x_des[-1][2].round(3)} {self.__force_from_sensor.value}', end = '\r', flush = True)
  
                # if dX_cur is not None:
                #     if np.linalg.norm((X_d-X_cur)) < eps and np.linalg.norm(dX_d - dX_cur) < eps:
                #         # print("Mech")
                #         pass
                        # raise(KeyboardInterrupt)
                        # self.rob.speedx("speedl",[0,0,0,0,0,0], self.__max_operational_acc, 1)
                        # raise(KeyboardInterrupt)


        except KeyboardInterrupt:
            self.rob_c.speedL([0,0,0,0,0,0], self.__max_operational_acc, 1)
            print('Robot is stopped')
            fig, ax = plt.subplots(nrows=4, ncols=1)
            plt.suptitle("Manipulator control, k = " + str(self.__k))
            ax[0].plot(time, [x[0] for x in x_act], label="actual x")
            ax[0].plot(time, [x[0] for x in x_des], label="needed x")
            ax[0].set_title("X coordinate")
            ax[0].set_xlabel("time, sec")
            ax[0].set_ylabel("position, m")
            ax[0].legend(loc="upper right")

            ax[1].plot(time, [x[1] for x in x_act], label="actual y")
            ax[1].plot(time, [x[1] for x in x_des], label="needed y")
            ax[1].set_title("Y coordinate")
            ax[1].set_xlabel("time, sec")
            ax[1].set_ylabel("position, m")
            ax[1].legend(loc="upper right")

            ax[2].plot(time, [x[2] for x in x_act], label="actual z")
            ax[2].plot(time, [x[2] for x in x_des], label="needed z")
            ax[2].set_title("Z coordinate")
            ax[2].set_xlabel("time, sec")
            ax[2].set_ylabel("position, m")
            ax[2].legend(loc="upper right")
            
            
            ax[3].plot(time, force_data, label="cur_force")
            ax[3].set_title("Force data")
            ax[3].set_xlabel("time, sec")
            ax[3].set_ylabel("force, N")
            ax[3].legend(loc="upper right")
            plt.show()

    def touch_ground_with_constant_force():
        pass

    def roll_with_constant_force():
        pass
    

    def __read_data_from_force_sensor(self):
        while True:
            rec_val =  self.futek.readData(write_to_file=1)
            if rec_val > 1:
                self.__force_from_sensor.value = rec_val
                # print(rec_val)
            else:
                self.__force_from_sensor.value = 0


    def basic_start(self, updz = 0.008):
        starting_pos = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]
        print("start freedrive")
        self.freedrive_transient(5)
        print("end freedrive")
        sensor_init_pose = self.rob_r.getActualTCPPose()
        print(sensor_init_pose)
        sensor_init_pose = sensor_init_pose[:3] + self._common_orient
        sensor_init_pose[2]=sensor_init_pose[2]+updz
        print(sensor_init_pose)
        self.lin(starting_pos)
        return sensor_init_pose, starting_pos

if __name__ == '__main__':
    ur_robot = UR5e_force(enable_force = 1)
    # print(ur_robot.rob)
    X_g, b = ur_robot.basic_start(updz = 0)
    print(f'"\n" {"X_g -"} {X_g} {"b - "} {b} {"before"}')
    X_start = X_g.copy()
    X_start[2] = 0.2
    ur_robot.lin(X_start)
    print(X_g)
    p1 = Process(target=ur_robot._force_planner, args=(X_g, [0,0,0],900))
    p1.start()
    # p2 = Process(target=ur_robot.vel_control)
    ur_robot.vel_control()

    print("done")


