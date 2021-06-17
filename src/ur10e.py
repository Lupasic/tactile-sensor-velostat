#!/usr/bin/python3

import urx
import time
from math import pi
from futek import Futek
import numpy as np

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf


sensor_l = 15
sensor_w = 15

class UR10:
    def __init__(self, address="172.31.1.25", debug = 1, enable_force=True):
        self._debug = debug
        self.enable_force = enable_force
        self.rob = urx.Robot(address, use_rt=False)
        if enable_force:
            self.futek = Futek(debug=1)
        self._acc = 0.15
        self._vec = 0.1
        # todo
        self._common_orient=[3.14,0.1,0]
    
    def touch_ground(self, needed_force=98,vel=0.01):
        cur_pose = self.rob.getl()
        # z axis value should be big
        cur_pose[2] -= 0.1
        # cur_pose[3] = -cur_pose[3]
        # cur_pose[4] = -cur_pose[4]
        if self._debug:
            print(cur_pose)
        self.rob.movel(cur_pose, acc=self._acc, vel=vel, wait=False, relative=False, threshold=None)
        while True:
            if self.futek.readData(write_to_file=1)>=needed_force:
                self.rob.stop()
                self.futek.readData(write_to_file=1,msg="touch_grd")
                print("End-effector touched ground")
                break

    def up(self, wait=True,dz=0.01):
        if wait:
            self.rob.up(z=dz, acc=self._acc, vel=self._vec)
        else:
            cur_pose = self.rob.getl()
            cur_pose[2] += dz
            # cur_pose[3] = -cur_pose[3]
            # cur_pose[4] = -cur_pose[4]
            if self._debug:
                print(cur_pose)
            self.rob.movel(cur_pose, acc=self._acc, vel=self._vec, wait=False, relative=False, threshold=None)
    
    def arc(self):
        pass

    def forward(self, dx, dy, wait=True):
        self.rob.movel([-dx,-dy,0,0,0,0], acc=self._acc, vel=self._vec, wait=wait, relative=True, threshold=None)

    def point_load(self, sleep_time=0, needed_force=98, vel=0.01):
        self.touch_ground(needed_force=needed_force, vel=vel)
        time.sleep(sleep_time)
        self.futek.readData(write_to_file=1,msg="go_up_point")
    
    def rolling_load(self,dx,dy,needed_force=98,vel=0.01):
        # dx, dy in mm"
        dx = dx/1000
        dy = dy/1000

        self.touch_ground(needed_force=needed_force, vel=vel)
        self.forward(dx,dy)
        self.futek.readData(write_to_file=1,msg="go_up_roll")

    def test_flat_sensor_point_load(self, init_pose, l, w, lp, wp, sleep_time, repeats=3, needed_force=98,updz=0.01, touch_ground_vel=0.01):
        # l and w in mm"
        l = l/1000
        w = w/1000
        eps = 0

        if lp == 0:
            l=0
            eps=1
            lp=1

        if wp == 0:
            w=0
            eps=1
            wp=1

        self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
        if self.enable_force:
            self.futek.readData(write_to_file=1,msg="start_exp")
        for i in range(repeats):
            self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
            if self.enable_force:
                self.futek.readData(write_to_file=1,msg="start_"+str(i)+"_iteration")
            cur_points=[]
            # eps is needed for lp or wp eual to zero, then we will do only 0 point)
            for xx in np.arange(l/(2*lp),l+eps,l/lp+eps):
                for yy in np.arange(w/(2*wp),w+eps,w/wp+eps):
                    temp = np.array(init_pose)-np.array([xx,yy]+[0,0,0,0])
                    cur_points.append(list(temp))
            for cur_poss in cur_points:
                self.rob.movel(cur_poss, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
                self.point_load(sleep_time=sleep_time,needed_force=needed_force, vel=touch_ground_vel)

        
    def test_flat_sensor_rolling_load(self, init_pose, l, w, lp=0, wp=0, repeats=2, needed_force=98,updz=0.01, touch_ground_vel=0.01):
        # l and w in mm"
        moving_l=l
        l = l/1000
        w = w/1000
        eps = 0

        if lp == 0:
            l=0
            eps=1
            lp=1

        if wp == 0:
            w=0
            eps=1
            wp=1
        
        self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
        if self.enable_force:
            self.futek.readData(write_to_file=1,msg="start_exp")
        for i in range(repeats):
            self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
            if self.enable_force:
                self.futek.readData(write_to_file=1,msg="start_"+str(i)+"_iteration")
            cur_points=[]
            # eps is needed for lp or wp eual to zero, then we will do only 0 point)
            for xx in np.arange(l/(2*lp),l+eps,l/lp+eps):
                for yy in np.arange(w/(2*wp),w+eps,w/wp+eps):
                    temp = np.array(init_pose)-np.array([xx,yy]+[0,0,0,0])
                    cur_points.append(list(temp))
            for cur_poss in cur_points:
                self.rob.movel(cur_poss, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
                self.rolling_load(moving_l,0,needed_force=needed_force,vel=touch_ground_vel)

    def freedrive_transient(self, timeout=30):
        self.rob.set_freedrive(1, timeout=timeout)
        time.sleep(timeout)
            

def basic_start(ur10, updz = 0.008):
    starting_pos = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]
    print("start freedrive")
    ur10.freedrive_transient(15)
    print("end freedrive")
    sensor_init_pose = ur10.rob.getl()
    print(sensor_init_pose)
    sensor_init_pose = sensor_init_pose[:3] + ur10._common_orient
    sensor_init_pose[2]=sensor_init_pose[2]+updz
    print(sensor_init_pose)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    return sensor_init_pose, starting_pos

def exp_point_load(ur10, forces, lp, wp, touch_ground_vel=0.01):
    sensor_init_pose, starting_pos = basic_start(ur10,updz=0.006)
    for cur_force in forces:
        ur10.test_flat_sensor_point_load(sensor_init_pose,sensor_l,sensor_w,lp,wp,0,3,needed_force=ur10.futek.F2raw(cur_force), touch_ground_vel=touch_ground_vel)
        ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True, relative=False)
    ur10.futek.close()

def exp_point_load3exp(ur10, forces, lp, wp, touch_ground_vel=0.001):
    sensor_init_pose, starting_pos = basic_start(ur10,updz=0.0052)
    for cur_force in forces:
        ur10.test_flat_sensor_point_load(sensor_init_pose,sensor_l,sensor_w,lp,wp,0,1,needed_force=ur10.futek.F2raw(cur_force), touch_ground_vel=touch_ground_vel)
        ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.futek.close()


def exp_rolling_load(ur10, touch_ground_vel=0.01):
    sensor_init_pose, starting_pos = basic_start(ur10)
    ur10.test_flat_sensor_rolling_load(sensor_init_pose, sensor_l,sensor_w,0,1,2)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.futek.close()

def exp_multi_rolling(ur10,repeats,force, touch_ground_vel=0.01):
    force = ur10.futek.F2raw(force)
    sensor_init_pose, starting_pos = basic_start(ur10)
    ur10.test_flat_sensor_rolling_load(sensor_init_pose, sensor_l,sensor_w,0,1,repeats,needed_force=force, touch_ground_vel=touch_ground_vel)


def exp_point_load_one_point(ur10,force, touch_ground_vel=0.01):
    sensor_init_pose, starting_pos = basic_start(ur10)
    force = ur10.futek.F2raw(force)
    ur10.test_flat_sensor_point_load(sensor_init_pose,sensor_l,sensor_w,0,0,1,3,needed_force=force, touch_ground_vel=touch_ground_vel)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.futek.close()




if __name__ == '__main__':
    ur10 = UR10(enable_force=1)
    print(ur10.rob)
    # EXP 1
    exp_point_load(ur10,[20, 40, 60],0,0, touch_ground_vel=0.001)

    # EXP 2
    # exp_point_load(ur10,[10,20,30,40,50,60,70,80],0,0, touch_ground_vel=0.001)

    # EXP 3
    # exp_point_load3exp(ur10,[20],4,4)
    # exp_point_load3exp(ur10,[pressure*S_big_spike],4,4)

    # EXP 4
    # exp_multi_rolling(ur10,10,40)
    
    #EXP 2
    S_small_spike = pi*0.0027**2/4
    S_big_spike = pi*0.012**2/4
    pressure = 500000
    # exp_point_load_one_point(ur10,pressure*S_small_spike)
    # exp_point_load_one_point(ur10,pressure*S_big_spike)
    
    # EXP 3
    # exp_point_load3exp(ur10,[pressure*S_small_spike],4,4)
    # exp_point_load3exp(ur10,[pressure*S_big_spike],4,4)
    
    

