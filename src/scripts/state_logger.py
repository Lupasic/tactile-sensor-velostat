import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize
from numpy import array, ones, zeros,exp
from setup_logger import logger
from os.path import isfile
from scipy.signal import find_peaks
from scipy.optimize import least_squares

class StateLogger:
    def __init__(self) -> None:
        self.x_act = []
        self.x_des = []
        self.dx_act= []
        self.dx_des = []
        self.time = []
        self.force_data = []
        # logger.info("All data meow")


    def read_data_from_futek_or_velostat(self, folder_name, pure_file_name): 
        starting_exp_timestamp = 0.0
        starting_exp_timestamp_file_path = "/home/app/tactile_sensor_velostat/experimental_data/starting_experiment_"+pure_file_name+".txt"
        if isfile(starting_exp_timestamp_file_path):
            tex_file = open(starting_exp_timestamp_file_path,mode="r")
            starting_exp_timestamp = float(tex_file.readline())

        logger.debug("I took %s folder with %s name",folder_name, pure_file_name)
        t = []
        data = []
        text = []
        temp = "/home/app/tactile_sensor_velostat/experimental_data/" + folder_name
        file_name = temp + "/exp_" + pure_file_name + ".txt"
        print("name is " + file_name)
        text_file = open(file_name,"r")

        for line in text_file:
            full_line_data = line.split(" ")
            if float(full_line_data[1]) < starting_exp_timestamp:
                continue
            data.append(float(full_line_data[0]))
            t.append(float(full_line_data[1]))
            if len(full_line_data) > 2:
                text.append(full_line_data[2])
            else:
                text.append("")
        return t, data, text

    def read_all_data_for_drawing_froce_comp(self, name):
        t_vel, data_vel, text_vel = self.read_data_from_futek_or_velostat("velostat_data",name)
        t_futek, data_futek, text_futek =self.read_data_from_futek_or_velostat("futek_data",name)
        logger.debug(len(t_vel))
        return [[t_vel,t_futek],[data_vel,data_futek],[text_vel,text_futek]]


    def fun(self, x, t, y):
        p = 0.1
        t_0 = 0

        k_p = x[1]*exp(-x[2]*p)
        tau_res = x[4]+x[5]*exp(-p/x[6])

        return x[0] + p*(k_p+x[3]*(1-exp(-(t-t_0)/tau_res)))*(1-exp(-x[7]/p)) - y

    def fun_test(self, x, t):
        p = 0.1
        t_0 = 0

        k_p = x[1]*exp(-x[2]*p)
        tau_res = x[4]+x[5]*exp(-p/x[6])

        return x[0] + p*(k_p+x[3]*(1-exp(-(t-t_0)/tau_res)))*(1-exp(-x[7]/p))

    def draw_velostat_data_with_fitting(self, t, data):
        data = array(data)
        data = 1/data
        t = array(t)
        t = t - t[0]
        x_kek = [0.047,2.592,0.693,0.736,0.4,16.5,0.4,2.0]
        x_init = x_kek.copy()

        res_robust = least_squares(self.fun, x_init, loss="soft_l1", f_scale= 0.1, args=(t, data))
        print(res_robust.x)

        predict_data = self.fun_test([*res_robust.x],t)
        # predict_data = self.fun_test([-0.19869451,2.20873776,0.93442205,0.01167769,-5.08459372,11.24773046, -4.43790163,2.9861408 ],t)

        plt.plot(t, data, 'o', label='data')
        plt.plot(t, predict_data, label='least square')
        plt.xlabel('$t$')
        plt.ylabel('$y$')
        plt.legend()
        plt.show()

    def data_preprocessing_for_force_comparison(self, whole_list):
        # Normalize futek data
        temp = array(whole_list[1][1])
        i = 0
        for el in temp:
            if el < 0:
                temp[i] = 0
            i = i+1
        
        norm_futek = normalize([temp],norm="max")
        whole_list[1][1]=norm_futek[0]

        # inverse and normalize velostat data
        # when I put 10 gramms on the sensor
        VELOSTAT_THRESHOLD = 1300
        temp = array(whole_list[1][0])
        i = 0
        for el in temp:
            if el >= 850:
                temp[i] = 1e+5
            i = i+1
        temp = 1/temp
        norm_vel = normalize([temp],norm="max")
        whole_list[1][0] = norm_vel[0]
        return whole_list


    def find_peaks(self, x, dist=1000, prom=0.5, width=500, thresh=0.4):
        """Find peaks

        Args:
            x (list): Velostat or futek data
        """
        peaks_val =[]
        peaks, _ = find_peaks(x, distance=dist)
        peaks_val.append(x[peaks])
        print(f'Peaks value for distance {x[peaks]}')
        peaks2, _ = find_peaks(x, prominence=prom)      # BEST!
        peaks_val.append(x[peaks2])
        print(f'Peaks value for prominence {x[peaks2]}')
        peaks3, _ = find_peaks(x, width=width)
        peaks_val.append(x[peaks3])
        print(f'Peaks value for width {x[peaks3]}')
        peaks4, _ = find_peaks(x, threshold=thresh)     # Required vertical distance to its direct neighbouring samples, pretty useless
        peaks_val.append(x[peaks4])
        print(f'Peaks value for threshold {x[peaks4]}')
        plt.subplot(2, 2, 1)
        plt.plot(peaks, x[peaks], "xr"); plt.plot(x); plt.legend(['distance'])
        plt.subplot(2, 2, 2)
        plt.plot(peaks2, x[peaks2], "ob"); plt.plot(x); plt.legend(['prominence'])
        plt.subplot(2, 2, 3)
        plt.plot(peaks3, x[peaks3], "vg"); plt.plot(x); plt.legend(['width'])
        plt.subplot(2, 2, 4)
        plt.plot(peaks4, x[peaks4], "xk"); plt.plot(x); plt.legend(['threshold'])
        plt.show()
        # p = input("Choose the best solution, start from 0 \n")
        return peaks_val[int(p)]

    def bar_chart_3d(self,peaks=None):
        fig = plt.figure()
        ax1 = fig.add_subplot(111, projection='3d')

        x3 = [1,1,1,2,2,2,3,3,3]
        y3 = [1,2,3,1,2,3,1,2,3]
        z3 = zeros(9)

        dx = ones(9)
        dy = ones(9)
        dz = [1.,0.55244635,0.60160475,0.71310662,0.25917747,0.96433319,
         0.46526515,0.49141629,0.54094081]

        ax1.bar3d(x3, y3, z3, dx, dy, dz,  edgecolor='blue',color=['black', 'red', 'green', 'blue', 'cyan','black', 'red', 'green', 'blue'])


        ax1.set_xlabel('x axis')
        ax1.set_ylabel('y axis')
        ax1.set_zlabel('z axis')

        plt.show()

    def find_pikes_from_velostat_and_futek(self, p):
        # futek_pikes = self.find_peaks(p[1][1])
        velostat_pikes = self.find_peaks(p[1][0],dist=500)
        return velostat_pikes, futek_pikes

    def draw_force_comparison(self, whole_list):
        fig, ax = plt.subplots(nrows=3, ncols=1, constrained_layout=True)
        # plt.subplots_adjust(top=0.925,bottom=0.1,left=0.155,
        # right=0.845,hspace=0.4,wspace=0.2)

        plt.suptitle("Force comparison")
        ax[0].plot(whole_list[0][1], whole_list[1][1])
        ax[0].set_title("Futek sensor data")
        ax[0].set_xlabel("time, sec")
        ax[0].set_ylabel("Force, value")
        # ax[1].legend(loc="upper right")
        
        
        ax[1].plot(whole_list[0][0], whole_list[1][0])
        ax[1].set_title("Velostat sensor data")
        ax[1].set_xlabel("time, sec")
        ax[1].set_ylabel("Force, value")
        # ax[0].legend(loc="upper right")

        ax[2].plot(whole_list[0][1], whole_list[1][1], label="futek data")
        ax[2].plot(whole_list[0][0], whole_list[1][0], label="velostat data")
        ax[2].set_title("Velostat and Futek data on the same plot")
        ax[2].set_xlabel("time, sec")
        ax[2].set_ylabel("Force, value")
        ax[2].legend(loc="upper right")
        plt.show()


    def receive_data(self, x_act, x_des, time, force_data, dx_act, dx_des):
        self.x_act.append(x_act)
        self.x_des.append(x_des)
        self.time.append(time)
        self.force_data.append(force_data)
        self.dx_act.append(dx_act)
        self.dx_des.append(dx_des)

    def draw_my_plots(self):
        self.draw_plots(self.x_act,self.x_des,self.time,self.force_data, self.dx_act,self.dx_des)

    def draw_plots(self, x_act, x_des, time, force_data, dx_act, dx_des):
        fig, ax = plt.subplots(nrows=4, ncols=2)
        plt.suptitle("Manipulator control")
        ax[0, 0].plot(time, [x[0] for x in x_act], label="actual x")
        ax[0, 0].plot(time, [x[0] for x in x_des], label="needed x")
        ax[0, 0].set_title("Position, X axis")
        ax[0, 0].set_xlabel("time, sec")
        ax[0, 0].set_ylabel("position, m")
        ax[0, 0].legend(loc="lower right")

        ax[1,0].plot(time, [x[1] for x in x_act], label="actual y")
        ax[1,0].plot(time, [x[1] for x in x_des], label="needed y")
        ax[1,0].set_title("Position, Y axis")
        ax[1,0].set_xlabel("time, sec")
        ax[1,0].set_ylabel("position, m")
        ax[1,0].legend(loc="lower right")

        ax[2,0].plot(time, [x[2] for x in x_act], label="actual z")
        ax[2,0].plot(time, [x[2] for x in x_des], label="needed z")
        ax[2,0].set_title("Position, Z axis")
        ax[2,0].set_xlabel("time, sec")
        ax[2,0].set_ylabel("position, m")
        ax[2,0].legend(loc="lower right")

        ax[3,0].plot(time, force_data, label="cur_force")
        ax[3,0].set_title("Force data")
        ax[3,0].set_xlabel("time, sec")
        ax[3,0].set_ylabel("force, N")
        ax[3,0].legend(loc="lower right")

        ax[0, 1].plot(time, [x[0] for x in dx_act], label="actual x")
        ax[0, 1].plot(time, [x[0] for x in dx_des], label="needed x")
        ax[0, 1].set_title("Velocity, X axis")
        ax[0, 1].set_xlabel("time, sec")
        ax[0, 1].set_ylabel("velocity, m/s")
        ax[0, 1].legend(loc="lower right")

        ax[1,1].plot(time, [x[1] for x in dx_act], label="actual y")
        ax[1,1].plot(time, [x[1] for x in dx_des], label="needed y")
        ax[1,1].set_title("Velocity, Y axis")
        ax[1,1].set_xlabel("time, sec")
        ax[1,1].set_ylabel("velocity, m/s")
        ax[1,1].legend(loc="lower right")

        ax[2,1].plot(time, [x[2] for x in dx_act], label="actual z")
        ax[2,1].plot(time, [x[2] for x in dx_des], label="needed z")
        ax[2,1].set_ylim(-0.1,0.1)
        ax[2,1].set_title("Velocity, Z axis")
        ax[2,1].set_xlabel("time, sec")
        ax[2,1].set_ylabel("velocity, m/s")
        ax[2,1].legend(loc="lower right")
        plt.show()

if __name__ == '__main__':
    state_logger = StateLogger()
    # logger.debug("kek")
    # p = state_logger.read_all_data_for_drawing_froce_comp("pike4_sensor2_exp1")
    # p = state_logger.data_preprocessing_for_force_comparison(p)
    # # velostat_peaks, futek_peaks = state_logger.find_pikes_from_velostat_and_futek(p)
    # # state_logger.bar_chart_3d()
    # state_logger.draw_force_comparison(p)
    t,data,_ = state_logger.read_data_from_futek_or_velostat("velostat_data","data_for_least_square_sensor10.1")
    state_logger.draw_velostat_data_with_fitting(t,data)