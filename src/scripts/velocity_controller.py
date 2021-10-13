from multiprocessing.sharedctypes import Value
from time import perf_counter, sleep
from multiprocessing import Manager, Process
# import numpy as np
from numpy import linalg, array, product
import logging

class VelocityController:
    def __init__(self) -> None:
        logging.basicConfig(level=logging.DEBUG)
        self.log = logging.getLogger("VelocityController")
        self.log.setLevel("DEBUG")
        self.init_states()
        self.init_constants()
        pass

    def init_states(self):
        self.cur_state = Manager().Namespace()
        self.cur_state.X_cur = None
        self.cur_state.Xd = None
        self.cur_state.dXd = None

        self.new_state = Manager().Namespace()
        self.new_state.U = None

    def init_constants(self):
        self.FREQ = 500
        self.K = 0.99

    def set_cur_state(self, planner_data, robot_data):
        """Set current state from robot and sensor data

        Args:
            planner_data (Manager().Namespace()): should consist of X des, dX des
            robot_data (Manager().Namespace()): should consist of X current
        """        
        self.cur_state.Xd = planner_data.Xd
        self.cur_state.dXd = planner_data.dXd
        self.cur_state.X_cur = robot_data.X_cur



    def get_new_velocity(self):
        """Return desired velocities after calculating control

        Returns:
            np.Array: velosities in x, y and z axes and [0,0,0] in orientation
        """        
        return list(self.new_state.U) + [0,0,0]


    def vel_control(self):
        t0 = perf_counter()
        t1 = 0
        i = 0
        time = []
        while True:
            if self.cur_state.dXd == None or self.cur_state.Xd == None \
                or self.cur_state.X_cur == None:
                continue
            t = perf_counter() - t0
            self.new_state.U = array(self.cur_state.dXd) + self.K*(array(self.cur_state.Xd) - array(self.cur_state.X_cur[:3]))
            if t -t1 < 1/self.FREQ:
                sleep(1/self.FREQ - (t -t1))
            

    def run_controller(self):
        """Start controller in a distinguish process
        """
        self.process = Process(target=self.vel_control)
        self.process.start()
        while not self.process.is_alive():
            continue
    
    def shutdown_controller(self):
        """Terminate process and clear other stuff
        """
        self.process.terminate()
        self.log.debug("Controller process was terminated")

if __name__ == '__main__':
    controller = VelocityController()
    controller.run_controller()
    planner_data = Manager().Namespace()
    planner_data.Xd = [1.0,2.0,3.0]
    planner_data.dXd = [0,0,1]
    robot_data = Manager().Namespace()
    robot_data.X_cur = [1,5,3,4,5,6]


    controller.set_cur_state(planner_data,robot_data)
    sleep(2)
    print(controller.get_new_velocity())
    controller.shutdown_controller()
    print("Done")
