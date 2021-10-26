from futek_sensor import FutekSensor
from time import perf_counter, sleep
from numpy import e, linalg, array
from multiprocessing import Manager, Process
import rtde_control
import rtde_receive
import logging
from force_planner import ForcePlanner
from state_logger import StateLogger
from velocity_controller import VelocityController

# https://github.com/LukeSkypewalker/URX-jupiter-notebook/blob/master/URX_notebook.ipynb

# https://s3-eu-west-1.amazonaws.com/ur-support-site/105198/scriptManual_SW5.10.pdf

# https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#speedj-example

# Read more about managers

class UR5e:
    def __init__(self, address="172.31.1.25", enable_force = 1, file_name=None, folder_name="futek_data"):
        logging.basicConfig(level=logging.DEBUG)
        self.log = logging.getLogger("UR5e")
        self.log.setLevel("DEBUG")
        self.rob_c = rtde_control.RTDEControlInterface("172.31.1.25")
        self.enable_force = enable_force
        self.fl = 0
        
        self.init_constants()
        self.init_states()

        self.process_rob_r = Process(target=self.read_data_from_manip,args=(address,))
        self.process_rob_r.start()

        if enable_force:
            futek = FutekSensor(debug=0,file_name=file_name, folder_name=folder_name)
            self.process_futek = Process(target=self.read_data_from_force_sensor,args=(futek,))
            self.process_futek.start()
        

    def init_states(self):
        self.cur_state = Manager().Namespace()
        self.cur_state.X_cur = None
        self.cur_state.dX_cur = None
        self.cur_state.F_cur = None

    def init_constants(self):
        self.FREQ = 500
        self.MAX_OPERATIONAL_ACC = 0.4
        self.MAX_LIN_ACC = 0.5
        self.MAX_LIN_VEL = 0.5

        self.FORCE_THRESHOLD = 1
        self.COMMON_ORIENT=[3.14,0.1,0]
        self.STARTING_POS = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]

    def read_data_from_manip(self,address):
        rob_r = rtde_receive.RTDEReceiveInterface(address)
        while True:
            self.cur_state.X_cur =  rob_r.getActualTCPPose()
            self.cur_state.dX_cur =  rob_r.getActualTCPSpeed()
            # if self.log.level == logging.DEBUG:
            #     print(f'{self.X_cur_shared[:]}',flush=True,end='\r')

    def read_data_from_force_sensor(self,futek):
        while True:
            rec_val =  futek.readData(write_to_file=1)
            if rec_val > self.FORCE_THRESHOLD:
                self.cur_state.F_cur = rec_val
            else:
                self.cur_state.F_cur = 0
            # print(f'{self.cur_state.F_cur} {rec_val}',end='\r',flush=True)

    def lin(self, target_pos, wait=True):
        if wait:
            asyncc = False
        else:
            asyncc = True    
        self.rob_c.moveL(target_pos, self.MAX_LIN_VEL, self.MAX_LIN_ACC, asyncc)

    def freedrive_transient(self, timeout=20):
        self.rob_c.teachMode()
        sleep(timeout)
        self.rob_c.endTeachMode()

    def shutdown_robot(self):
        if self.enable_force:
            self.process_futek.terminate()
        self.process_rob_r.terminate()
        if self.fl:
            self.vel_controller.shutdown_controller()
            self.force_planner.shutdown_planner()
        self.log.debug("Robot process was terminated")

    
    def up(self, wait=True,dz=0.01):
        if wait:
            asyncc = False
        else:
            asyncc = True  
        cur_pose = self.cur_state.X_cur
        cur_pose[2] += dz
        # self.lin_force(cur_pose[:3])
        self.rob_c.moveL(cur_pose, self.MAX_LIN_VEL, self.MAX_LIN_ACC, asyncc)


    def basic_start(self, updz = 0.000):
        print("start freedrive")
        self.freedrive_transient(5)
        print("end freedrive")
        sensor_init_pose = self.cur_state.X_cur
        print(sensor_init_pose)
        sensor_init_pose = sensor_init_pose[:3] + self.COMMON_ORIENT
        sensor_init_pose[2] = sensor_init_pose[2]+updz
        print(sensor_init_pose)
        self.lin(self.STARTING_POS)
        return sensor_init_pose

    def check_finish_motion(self, Xd, dXd, Fd_real):
        eps = 1e-3
        a = linalg.norm(array(Xd[:3])-array(self.cur_state.X_cur[:3])) <= eps
        b = linalg.norm(array(dXd[:3]) - array(self.cur_state.dX_cur[:3])) <= eps
        c = abs(Fd_real - self.cur_state.F_cur) <= 3
        if  a and b and c:
            return True
        else:
            return False

    def run_env_for_lin_force(self):
        self.force_planner = ForcePlanner()
        self.force_planner.run_planner()
        self.vel_controller = VelocityController()
        self.vel_controller.run_controller()
        self.fl = 1

    def lin_force(self, Xg, dXg = [0,0,0], Fd_ideal=0, Fd_real = 0):
        state_logger = StateLogger()
        t0 = perf_counter()
        t1 = 0
        Xd = Xg
        dXd = dXg
        try:
            while not self.check_finish_motion(Xd,dXd,Fd_real):
                t = perf_counter() - t0 
                if t - t1 >=1/self.FREQ:
                    self.force_planner.set_cur_state(Xg[:3],dXg,Fd_ideal,self.cur_state.F_cur)
                    Xd, dXd = self.force_planner.get_new_state()
                    self.vel_controller.set_cur_state(Xd, dXd, self.cur_state.X_cur)
                    U = self.vel_controller.get_new_velocity()
                    self.rob_c.speedL(U,self.MAX_OPERATIONAL_ACC,1/self.FREQ)
                    # print(f'Cur state {array(self.cur_state.X_cur[2]).round(2)}, Desired state {array(Xd[2]).round(2)}')
                    print(f'{array(self.cur_state.X_cur[2]).round(3)} {array(Xd[2]).round(3)} {self.cur_state.F_cur} check_motion \
                        {linalg.norm(array(Xd)-array(self.cur_state.X_cur[:3]))} \
                        {linalg.norm(array(dXd) - array(self.cur_state.dX_cur[:3]))} \
                        {self.cur_state.F_cur} {self.check_finish_motion(Xd,dXd,Fd_real)}', end = '\r', flush = True)
                    state_logger.receive_data(self.cur_state.X_cur,Xd,t,self.cur_state.F_cur)
                    t1 = t
            # QUITE IMPORTANT BOOLSHIT otherwise, I cannot use other commands
            self.rob_c.reuploadScript()
            sleep(1)
            return state_logger
        except KeyboardInterrupt:
            self.rob_c.speedL([0,0,0,0,0,0], self.MAX_OPERATIONAL_ACC, 1)
            print('Robot is stopped')
            state_logger.draw_my_plots()

    def point_load(self,Xg,h_init=0.2, dXg = [0,0,0], Fd_ideal=0, Fd_real = 0):
        starting_pos = Xg.copy()
        starting_pos[2] = h_init
        self.lin(starting_pos)
        kek = self.lin_force(Xg,dXg=dXg,Fd_ideal=Fd_ideal,Fd_real=Fd_real)
        self.up(dz=0.2,wait=True)
        kek.draw_my_plots()

def test_non_force_func():
    robot = UR5e(enable_force=0)
    try:
        sensor_pos = robot.basic_start()
        print(array(robot.cur_state.X_cur))
        robot.up(dz=-0.2)
        print(array(robot.cur_state.X_cur))
        robot.shutdown_robot()
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

def test_force_planner():
    robot = UR5e(enable_force=1)
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.cur_state.X_cur 
        robot.point_load(sensor_pos,Fd_ideal=900,Fd_real=900)
        robot.shutdown_robot()
        print("I am here")
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

def test_point_load():
    robot = UR5e(enable_force=1)
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        robot.point_load(sensor_pos,Fd_ideal=900,Fd_real=900)
        robot.shutdown_robot()
        print("I am here")
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

if __name__ == '__main__':
    test_point_load()
    # test_force_planner()