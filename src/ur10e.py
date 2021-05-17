#!/usr/bin/python3

import urx
import time
from futek import Futek
import numpy as np
import math3d as m3d

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf

class UR10:
    def __init__(self, address="172.31.1.25", debug = 1, enable_force=True):
        self._debug = debug
        self.enable_force = enable_force
        self.rob = urx.Robot(address, use_rt=False)
        if enable_force:
            self.futek = Futek(debug=1)
        self._acc = 0.02
        self._vec = 0.02
        # todo
        self._common_orient=[3.14,0.1,0]
    
    def touch_ground(self, needed_force=98):
        cur_pose = self.rob.getl()
        # z axis value should be big
        cur_pose[2] -= 0.5
        # cur_pose[3] = -cur_pose[3]
        # cur_pose[4] = -cur_pose[4]
        if self._debug:
            print(cur_pose)
        self.rob.movel(cur_pose, acc=self._acc, vel=self._vec, wait=False, relative=False, threshold=None)
        while True:
            # try:
            temp = self.futek.readData(write_to_file=0)
            print(temp)
            if self.futek.readData(write_to_file=0)>=needed_force:
                print("End-effector touched ground")
                self.futek.readData(write_to_file=1,msg="touch_grd")
                self.rob.stop()
                break
            # except Exception:
            #     print("Corrupted")
            #     pass

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

    def point_load(self, sleep_time=0, needed_force=98):
        self.touch_ground(needed_force=needed_force)
        time.sleep(sleep_time)
        self.futek.readData(write_to_file=1,msg="go_up_point")
        # self.up()
    
    def rolling_load(self,dx,dy,needed_force=98):
        self.touch_ground(needed_force=needed_force)
        self.forward(dx,dy)
        self.futek.readData(write_to_file=1,msg="go_up_roll")
        # self.up()

    def test_flat_sensor_point_load(self, init_pose, l, w, lp, wp, sleep_time, repeats=3, needed_force=98,updz=0.04):
        # l and w in mm"
        l = l/1000
        w = w/1000
        
        init_pose[2]=init_pose[2]+updz
        self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
        if self.enable_force:
            self.futek.readData(write_to_file=1,msg="start_exp")
        for i in range(repeats):
            self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
            if self.enable_force:
                self.futek.readData(write_to_file=1,msg="start_"+str(i)+"_iteration")
            cur_points=[]
            for xx in np.arange(0,l+l/lp,l/lp):
                for yy in np.arange(0,w+w/wp,w/wp):
                    temp = np.array(init_pose)-np.array([xx,yy]+[0,0,0,0])
                    cur_points.append(list(temp))
            # if self._debug:
            #     print(cur_points)
            for cur_poss in cur_points:
                self.rob.movel(cur_poss, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
                # self.point_load(sleep_time=sleep_time,needed_force=needed_force)
                self.rolling_load(0.1,0)

    def freedrive_transient(self, timeout=30):
        self.rob.set_freedrive(1, timeout=timeout)
        time.sleep(timeout)
            


if __name__ == '__main__':
    starting_pos = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]
    ur10 = UR10(enable_force=1)
    print(ur10.rob)
    # for i in range(1):
    #     ur10.point_load()
    #     time.sleep(10)
    #     ur10.futek.readData(write_to_file=1,msg="kek1")
    #     time.sleep(5)
    #     ur10.futek.readData(write_to_file=1,msg="kek2")
    #     ur10.up(dz=0.01)
    #     ur10.futek.readData(write_to_file=1,msg="kek3")
    #     time.sleep(5)
    #     ur10.futek.readData(write_to_file=1,msg="kek4")
                
        # ur10.up(dz=0.05)
    print("start freedrive")
    ur10.freedrive_transient(15)
    print("end freedrive")
    sensor_init_pose = ur10.rob.getl()
    print(sensor_init_pose)
    sensor_init_pose = sensor_init_pose[:3] + ur10._common_orient
    print(sensor_init_pose)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.test_flat_sensor_point_load(sensor_init_pose,50,40,4,5,0,1)
    ur10.rob.movel(starting_pos,ur10._acc,ur10._vec,wait=True)
    ur10.forward(0.05,0)
    ur10.futek.close()
