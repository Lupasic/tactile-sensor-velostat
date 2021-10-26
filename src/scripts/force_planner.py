from time import perf_counter, sleep
from scipy.integrate import odeint
from numpy import linspace, array, linalg, isnan
from multiprocessing import Manager, Process
import logging

class ForcePlanner:
    def __init__(self) -> None:
        logging.basicConfig(level=logging.DEBUG)
        self.log = logging.getLogger("ForcePlanner")
        self.log.setLevel("DEBUG")
        self.init_states()
        self.init_constants()
        pass

    def init_states(self):
        self.cur_state = Manager().Namespace()
        self.cur_state.Xg = None
        self.cur_state.dXg = None
        self.cur_state.F_cur = None
        self.cur_state.Fd_ideal = None

        self.new_state = Manager().Namespace()
        self.new_state.Xd = None
        self.new_state.dXd = None

    def init_constants(self):
        self.VIR_STIFF = 400000
        self.FREQ = 500

    # def set_cur_state(self, robot_manager):
    #     """Set current state from robot and sensor data

    #     Args:
    #         robot_manager (Manager().Namespace()): should consist of X goal, dX goal and F desired
    #         force_sensor_data (int): data from force sensor
    #     """        
    #     self.cur_state.Xg = robot_manager.Xg
    #     self.cur_state.dXg = robot_manager.dXg
    #     self.cur_state.Fd_ideal = robot_manager.Fd_ideal
    #     self.cur_state.F_cur = robot_manager.F_cur
    #     print(self.cur_state.dXg)
    #     print("f")

    def set_cur_state(self, Xg,dXg,Fd_ideal,F_cur):
        """Set current state from robot and sensor data

        Args:
            robot_manager (Manager().Namespace()): should consist of X goal, dX goal and F desired
            force_sensor_data (int): data from force sensor
        """        
        self.cur_state.dXg = dXg
        self.cur_state.Fd_ideal = Fd_ideal
        self.cur_state.F_cur = F_cur
        if Xg is not None and Fd_ideal is not None:
            # Xg2 = Xg[2]
            Xg2 = Xg[2] - Fd_ideal/self.VIR_STIFF
            self.cur_state.Xg = [Xg[0],Xg[1], Xg2]


    def get_new_state(self):
        """Return desired positions and velocities after calculating in planner func

        Returns:
            np.Array: positions in x, y and z axes
            np.Array: velosities in x, y and z axes
        """
        while self.new_state.Xd is None or self.new_state.dXd is None:
            continue
        return self.new_state.Xd, self.new_state.dXd

    def f_x(self, delta_prev, t , F_cur): return F_cur - self.VIR_STIFF*delta_prev

    def solve_diff_eq_force_planner(self, delta_prev):
        """Solve differential equation for force planner

        Args:
            delta_prev (double): Consist of solution for Z axis from previous state
        """
        t0 = perf_counter()
        t = linspace(0,1/self.FREQ,3)
        delta = odeint(self.f_x, delta_prev, t, args = (self.cur_state.F_cur,))[-1][0]
        if isnan(delta):
            delta=0
        ddelta = self.cur_state.F_cur - self.VIR_STIFF * delta
        ttt = perf_counter() - t0
        return delta, ddelta, ttt       

    def force_planner_solution(self):
        """Based on current state, calculate a new desired state for robot.
        It stores in self.new_state var.
        """        
        Xg_cur = [0,0,0]
        t0 = perf_counter()
        t1 = 0
        delta_prev = 0
        e = 0.001
        fl = 0
        while True:
            # if self.log.level == logging.DEBUG:
                # print(f' Current {self.new_state.Xd}',flush=True, end = '\r')
            if self.cur_state.Xg == None or self.cur_state.dXg == None \
                or self.cur_state.Fd_ideal == None or self.cur_state.F_cur == None:
                continue
            # print(Xg_cur)

            # if linalg.norm(array(Xg_cur) - array(self.cur_state.Xg)) <= e or fl == 0:
            # if fl == 0:
            #     self.log.info(f' New point received {self.cur_state.Xg}')
            #     # modify z axis
            #     # Xg2 = self.cur_state.Xg[2] - self.cur_state.Fd_ideal/self.VIR_STIFF
            #     # print(f'Inside {Xg2} {self.cur_state.Xg[2]}')
            #     # self.cur_state.Xg = [self.cur_state.Xg[0],self.cur_state.Xg[1], Xg2]
            #     # Xg_cur = self.cur_state.Xg
            #     fl = 1
            t = perf_counter() - t0 
            if t - t1 >=1/self.FREQ:
                # modify trajectory_based on force
                delta, ddelta, __ = self.solve_diff_eq_force_planner(delta_prev)
                # in first iteration, __delta_prev_should be defined
                # print(f' {delta} {delta_prev}',flush=True, end = '\r')
                # print(f'Delta {delta} Xg_cur {self.cur_state.Xg[2]}')
                delta_prev = delta

                # modify X_d
                self.new_state.Xd = [self.cur_state.Xg[0], self.cur_state.Xg[1], self.cur_state.Xg[2] + delta]
                self.new_state.dXd = [self.cur_state.dXg[0], self.cur_state.dXg[1], self.cur_state.dXg[2] + ddelta]
                
                t1 = t

    def run_planner(self):
        """Start planner in a distinguish process
        """
        self.process = Process(target=self.force_planner_solution)
        self.process.start()
        while not self.process.is_alive():
            continue
    
    def shutdown_planner(self):
        """Terminate process and clear other stuff
        """
        self.process.terminate()
        self.log.debug("Planner process was terminated")

if __name__ == '__main__':
    planner = ForcePlanner()
    planner.run_planner()
    robot = Manager().Namespace()
    robot.Xg = [1.0,2.0,3.0]
    robot.dXg = [0,0,0]
    robot.F_cur = 30
    robot.Fd_ideal = 900
    planner.set_cur_state(robot)
    sleep(2)
    print(planner.get_new_state())
    planner.shutdown_planner()
    print("Done")
