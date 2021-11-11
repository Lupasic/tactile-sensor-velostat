from futek_sensor import FutekSensor
from time import perf_counter, sleep
from numpy import e, linalg, array, arange
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
        self.FREQ = 250
        self.MAX_OPERATIONAL_ACC = 0.3
        self.MAX_LIN_ACC = 0.5
        self.MAX_LIN_VEL = 0.5

        self.FORCE_THRESHOLD = 0.30
        self.COMMON_ORIENT=[3.14,0.1,0]
        self.STARTING_POS = [-0.6284734457983073, 0.04110124901844167, 0.24322792682955954, 2.885542842994124, -0.09630215284222861, -0.8197553730344054]

    def read_data_from_manip(self,address):
        """Special process for reading Pos and Speed from manipulator

        Args:
            address (string): IP address. The same as in robot
        """
        rob_r = rtde_receive.RTDEReceiveInterface(address)
        while True:
            self.cur_state.X_cur =  rob_r.getActualTCPPose()
            self.cur_state.dX_cur =  rob_r.getActualTCPSpeed()
            # if self.log.level == logging.DEBUG:
            #     print(f'{self.X_cur_shared[:]}',flush=True,end='\r')

    def read_data_from_force_sensor(self,futek):
        """Special process for reading data from futek sensor

        Args:
            futek (FutekSensor): class, where all functions for working with futek are determined
        """
        while True:
            rec_val =  futek.readData(write_to_file=1)
            if rec_val > self.FORCE_THRESHOLD:
                self.cur_state.F_cur = rec_val
            else:
                self.cur_state.F_cur = 0
            # print(f'{self.cur_state.F_cur} {rec_val}',end='\r',flush=True)

    def lin(self, target_pos, wait=True):
        """Straight movement in cartesian space. 
        This function based on high-level function from robot

        Args:
            target_pos (list): 6 components (3pos, 3 orient)
            wait (bool, optional): If False, then it will activate movement 
            and next code line will be executed. Defaults to True.
        """
        if wait:
            asyncc = False
        else:
            asyncc = True    
        self.rob_c.moveL(target_pos, self.MAX_LIN_VEL, self.MAX_LIN_ACC, asyncc)

    def freedrive_transient(self, timeout=20):
        """We can move robot manipulator by our own hands for timeout time

        Args:
            timeout (int, optional): in seconds. We can move the robot in timeout time. Defaults to 20.
        """
        self.rob_c.teachMode()
        sleep(timeout)
        self.rob_c.endTeachMode()

    def shutdown_robot(self):
        """Kill all processes (vel controll, force planner, read data from mapilator and futek)
        """
        if self.enable_force:
            self.process_futek.terminate()
        self.process_rob_r.terminate()
        if self.fl:
            self.vel_controller.shutdown_controller()
            self.force_planner.shutdown_planner()
        self.log.debug("Robot process was terminated")

    
    def up(self, wait=True,dz=0.01):
        """Linear movement in cartesian space, only in Z axis

        Args:
            wait (bool, optional): If False, then it will activate movement 
            and next code line will be executed. Defaults to True.
            dz (float, optional): in meters. the value of movement in Z axis. Defaults to 0.01.
        """
        if wait:
            asyncc = False
        else:
            asyncc = True  
        cur_pose = self.cur_state.X_cur
        cur_pose[2] += dz
        self.rob_c.moveL(cur_pose, self.MAX_LIN_VEL, self.MAX_LIN_ACC, asyncc)


    def basic_start(self, updz = 0.000):
        """We are starting freedrive for touching needed point via end effector.
        After this robot will return to initial position (predefined as a constant).
        End effector orientation is changing on default (parallel to Z axis).
        Function will return touched point.

        Args:
            updz (float, optional): offset in Z axis. Defaults to 0.000.

        Returns:
            list: position and orientation (6 values) of touched point. Orientation is parallel to Z axis.
        """
        print("start freedrive")
        self.freedrive_transient(10)
        print("end freedrive")
        sensor_init_pose = self.cur_state.X_cur
        print(sensor_init_pose)
        sensor_init_pose = sensor_init_pose[:3] + self.COMMON_ORIENT
        sensor_init_pose[2] = sensor_init_pose[2]+updz
        print(sensor_init_pose)
        self.lin(self.STARTING_POS)
        return sensor_init_pose

    def check_finish_motion(self, Xd, dXd, Fd_real):
        """It is needed for lin_force. We are checking can we finish the control.
        It is based on measuring error between current pos, vel and force.
        eps var is error size

        Args:
            Xd (list): Desired position. We are interested in only position (3 components)
            dXd (list): Desired velocities. We are interested in only linear velocities
            Fd_real (int): Force value

        Returns:
            bool: if True, we can finish the motion.
        """
        eps = 1e-3
        a = linalg.norm(array(Xd[:3])-array(self.cur_state.X_cur[:3])) <= eps
        b = linalg.norm(array(dXd[:3]) - array(self.cur_state.dX_cur[:3])) <= eps
        c = abs(Fd_real - self.cur_state.F_cur) <= 0.03
        if  a and b and c:
            return True
        else:
            return False

    def run_env_for_lin_force(self):
        """If we want to use lin_force, it is obligatory to use this function.
        It activates force planner and velocity controller.
        """
        self.force_planner = ForcePlanner()
        self.force_planner.run_planner()
        self.vel_controller = VelocityController()
        self.vel_controller.run_controller()
        self.fl = 1

    def lin_force(self, Xg, dXg = [0,0,0], Fd_ideal=0, Fd_real = 0):
        """LIN control with force as a constraint. Orientation won't change during the movement.

        Args:
            Xg (list): Goal position, only pos (3 components)
            dXg (list, optional): goal velocity. Defaults to [0,0,0].
            Fd_ideal (int, optional): Desired force. Because of math in force planner, we won't be reach ideal force. Defaults to 0.
            Fd_real (int, optional): What force shoud be reached. Should be smaller than ideal. Can be found empiricaly. Defaults to 0.

        Returns:
            [type]: [description]
        """
        state_logger = StateLogger()
        t0 = perf_counter()
        Xd = Xg
        dXd = dXg
        try:
            while not self.check_finish_motion(Xd,dXd,Fd_real):
                t = perf_counter() - t0 
                start = perf_counter()
                self.force_planner.set_cur_state(Xg[:3],dXg,Fd_ideal,self.cur_state.F_cur)
                Xd, dXd = self.force_planner.get_new_state()
                self.vel_controller.set_cur_state(Xd, dXd, self.cur_state.X_cur)
                U = self.vel_controller.get_new_velocity()
                self.rob_c.speedL(U,self.MAX_OPERATIONAL_ACC,1/self.FREQ)
                print(f'{array(self.cur_state.X_cur[2]).round(3)} {array(Xd[2]).round(3)} {self.cur_state.F_cur} check_motion \
                    {linalg.norm(array(Xd)-array(self.cur_state.X_cur[:3]))} \
                    {linalg.norm(array(dXd) - array(self.cur_state.dX_cur[:3]))} \
                    {self.cur_state.F_cur} {self.check_finish_motion(Xd,dXd,Fd_real)}', end = '\r', flush = True)
                # if self.cur_state.F_cur >= 0.4:   
                state_logger.receive_data(self.cur_state.X_cur,Xd,t,self.cur_state.F_cur, self.cur_state.dX_cur, U[:3])
                end = perf_counter()
                duration = end - start
                if duration < 1/self.FREQ:
                    sleep(1/self.FREQ - duration)
            # QUITE IMPORTANT BOOLSHIT otherwise, I cannot use other commands
            self.rob_c.reuploadScript()
            sleep(1)
            return state_logger
        except KeyboardInterrupt:
            self.rob_c.speedL([0,0,0,0,0,0], self.MAX_OPERATIONAL_ACC, 1)
            print('Robot is stopped')
            state_logger.draw_my_plots()

    def point_load(self,Xg,h_init=0.2, Fd_ideal=0, Fd_real = 0):
        """Touch Xg point with needed force and go up on initial height

        Args:
            Xg (list): Goal position. Without orientation
            h_init (float, optional): The height of manipulator before starting the lin force operation. Defaults to 0.2.
            Fd_ideal (int, optional): Desired force. Because of math in force planner, we won't be reach ideal force. Defaults to 0.
            Fd_real (int, optional): What force shoud be reached. Should be smaller than ideal. Can be found empiricaly. Defaults to 0.

        """
        starting_pos = Xg.copy()
        starting_pos[2] = h_init
        self.lin(starting_pos)
        kek = self.lin_force(Xg,Fd_ideal=Fd_ideal,Fd_real=Fd_real)
        self.up(dz=h_init,wait=True)
        kek.draw_my_plots()

    def sensor_point_load(self,Xg, l, w, lp=0, wp=0, repeats=1, h_init=0.1, Fd_ideal=0, Fd_real = 0):
        """Divide a rectangular (sides are l and w) on lp X wp amount of sectors and touch the center of each sector
        with needed force and go up on initial height.

        Args:
            Xg (list): Goal position. Without orientation.
            You should put end effector exactly in the beginning of rectangular. You should touch the ground.
            l (float): in mm. length of sensor
            w (float): in mm. width of rectangular area
            lp (int, optional): amount of sectors in length direction. Defaults to 0.
            wp (int, optional): amount of sectors in width direction. Defaults to 0.
            repeats (int, optional): Amount of experiment repeats. Defaults to 1.
            h_init (float, optional): The height of manipulator before starting the lin force operation. Defaults to 0.1.
            Fd_ideal (int, optional): Desired force. Because of math in force planner, we won't be reach ideal force. Defaults to 0.
            Fd_real (int, optional): What force shoud be reached. Should be smaller than ideal. Can be found empiricaly. Defaults to 0.
        """
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

        starting_pos = Xg.copy()
        starting_pos[2] = h_init
        for i in range(repeats):
            self.lin(starting_pos)
            cur_points=[]
            # eps is needed for lp or wp eual to zero, then we will do only 0 point)
            for xx in arange(w/(2*wp),(w)+eps,w/wp+eps):
                for yy in arange(l/(2*lp),(l)+eps,l/lp+eps):
                    temp = array(starting_pos)-array([xx,yy]+[0,0,0,0])
                    cur_points.append(list(temp))
            for cur_poss in cur_points:
                    self.lin(cur_poss)
                    kek = self.lin_force([cur_poss[0],cur_poss[1],Xg[2]],Fd_ideal=Fd_ideal,Fd_real=Fd_real)
                    self.up(dz=0.01)
                    kek.draw_my_plots()
                    self.lin(cur_poss)

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
        robot.point_load(sensor_pos,Fd_ideal=900,Fd_real=500)
        robot.shutdown_robot()
        print("I am here")
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

def test_point_load():
    robot = UR5e(enable_force=1,file_name="test_point_load")
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        robot.point_load(sensor_pos,Fd_ideal=50,Fd_real=5000,h_init=0.08)
        robot.shutdown_robot()
        print("I am here")
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

def test_sensor_multi_touch():
    robot = UR5e(enable_force=1)
    try:
        robot.run_env_for_lin_force()
        sleep(1)
        sensor_pos = robot.basic_start()
        robot.sensor_point_load(sensor_pos,17,14,lp=3,wp=3,repeats=2,Fd_ideal=20,Fd_real=10)
        # robot.point_load(sensor_pos,Fd_ideal=150,Fd_real=100)
        # robot.point_load(sensor_pos,Fd_ideal=500,Fd_real=400)
        robot.shutdown_robot()
        print("I am here")
    except KeyboardInterrupt:
        robot.rob_c.speedL([0,0,0,0,0,0], robot.MAX_OPERATIONAL_ACC, 1)
        print('Robot is stopped')

if __name__ == '__main__':
    test_point_load()
    # test_sensor_multi_touch()

    # test_force_planner()