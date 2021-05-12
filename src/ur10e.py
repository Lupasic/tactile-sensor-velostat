#!/usr/bin/python3

import urx
import time
from futek import Futek
import numpy as np
import math3d as m3d

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf

class UR10:
    def __init__(self, address="172.31.1.25", debug = 1):
        self._debug = debug
        self.rob = urx.Robot(address, use_rt=False)
        self.futek = Futek(debug=0)
        self._acc = 0.01
        self._vec = 0.01
        # todo
        self._common_orient=[1,1,1]
    
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
            try:
                if self.futek.readData(write_to_file=0)>=needed_force:
                    print("End-effector touched ground")
                    self.futek.readData(write_to_file=1,msg="touch_grd")
                    self.rob
                    # robot.up(z=0.05, acc=0.01, vel=0.01)
                    break
            except Exception:
                print("Corrupted")
                pass

    def up(self, wait=True,dz=0.05):
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
        # cur_pose = self.rob.getl()
        # cur_pose[0] -= dx
        # cur_pose[1] -= dy
        # if self._debug:
        #     print(cur_pose)
        self.rob.movel([-dx,-dy,0,0,0,0], acc=self._acc, vel=self._vec, wait=wait, relative=True, threshold=None)

    def point_load(self, sleep_time=0, needed_force=98):
        self.touch_ground(needed_force=needed_force)
        time.sleep(sleep_time)
        self.futek.readData(write_to_file=1,msg="go_up_point")
        self.up()
    
    def rolling_load(self,dx,dy,needed_force=98):
        self.touch_ground(needed_force=needed_force)
        self.forward(dx,dy)
        self.futek.readData(write_to_file=1,msg="go_up_roll")
        self.up()

    def test_flat_sensor_point_load(self, init_pose, l, w, sleep_time, lp, wp, repeats=3, needed_force=98,updz=0.05):
        # l and w in mm"
        l = l/1000
        w = w/1000
        
        init_pose[2]=init_pose+updz
        self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
        self.futek.readData(write_to_file=1,msg="start_exp")
        for i in range(repeats):
            self.rob.movel(init_pose, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
            self.futek.readData(write_to_file=1,msg="start_"+i+"_rep")
            cur_points=[]
            for xx in range(0,lp,l/lp):
                for yy in range(0,wp,w/wp):
                    temp = np.array(init_pose)+np.array([xx,yy]+init_pose[2:])
                    cur_points.append(list(temp))
            if self._debug:
                print(cur_points)
            for cur_poss in cur_points:
                self.rob.movel(cur_poss, acc=self._acc, vel=self._vec, wait=True, relative=False, threshold=None)
                self.point_load(sleep_time=sleep_time,needed_force=needed_force)
            




if __name__ == '__main__':
    # address="172.31.1.25"
    # rob = urx.Robot(address, use_rt=False)
    ur10 = UR10()
    print(ur10.rob)
    sensor = Futek(debug=0)
    print(sensor.readData())
    print("kek")

    pose = robot.getl()
    pose[2] -= 0.5
    # pose[3] = -pose[3]
    # pose[4] = -pose[4]
    print(pose)
    robot.movel(pose, acc=0.01, vel=0.01, wait=False, relative=False, threshold=None)
    while True:
        try:
            if sensor.readData(write_to_file=0)>98:
                print("i am here")
                sensor.readData(write_to_file=1)
                robot.stop()
                robot.up(z=0.05, acc=0.01, vel=0.01)
                break
        except Exception:
            print("Bleat")
            pass
    print("hooray")
        
