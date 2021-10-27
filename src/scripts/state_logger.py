
import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize
from numpy import array
from setup_logger import logger

class StateLogger:
    def __init__(self) -> None:
        self.x_act = []
        self.x_des = []
        self.time = []
        self.force_data = []
        logger.info("All data meow")


    def read_data_from_futek_or_velostat(self, folder_name, pure_file_name):
        logger.debug("I took %s folder with %s name",folder_name, pure_file_name)
        t = []
        data = []
        text = []
        temp = "../experimental_data/" + folder_name
        file_name = temp + "/exp_" + pure_file_name + ".txt"
        print("name is " + file_name)
        text_file = open(file_name,"r")

        for line in text_file:
            full_line_data = line.split(" ")
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

    def data_preprocessing_for_force_comparison(self, whole_list):
        # Normalize futek data
        temp = array(whole_list[1][1])
        norm_futek = normalize([temp],)
        whole_list[1][1]=norm_futek[0]

        # inverse and normalize velostat data
        temp = array(whole_list[1][0])
        temp = 1/temp
        norm_vel = normalize([temp])
        whole_list[1][0] = norm_vel[0]

        return whole_list



    def draw_force_comparison(self, whole_list):
        fig, ax = plt.subplots(nrows=3, ncols=1)
        plt.suptitle("Force comparison")
        ax[0].plot(whole_list[0][0], whole_list[1][0])
        ax[0].set_title("Velostat sensor data")
        ax[0].set_xlabel("time, sec")
        ax[0].set_ylabel("Force, value")
        # ax[0].legend(loc="upper right")

        ax[1].plot(whole_list[0][1], whole_list[1][1])
        ax[1].set_title("Futek sensor data")
        ax[1].set_xlabel("time, sec")
        ax[1].set_ylabel("Force, value")
        # ax[1].legend(loc="upper right")

        ax[2].plot(whole_list[0][0], whole_list[1][0], label="velostat data")
        ax[2].plot(whole_list[0][1], whole_list[1][1], label="futek data")
        ax[2].set_title("Velostat and Futek data on the same plot")
        ax[2].set_xlabel("time, sec")
        ax[2].set_ylabel("Force, value")
        ax[2].legend(loc="upper right")
        plt.show()


    def receive_data(self, x_act, x_des, time, force_data):
        self.x_act.append(x_act)
        self.x_des.append(x_des)
        self.time.append(time)
        self.force_data.append(force_data)

    def draw_my_plots(self):
        self.draw_plots(self.x_act,self.x_des,self.time,self.force_data)

    def draw_plots(self, x_act, x_des, time, force_data):
        fig, ax = plt.subplots(nrows=4, ncols=1)
        plt.suptitle("Manipulator control, k = ")
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

if __name__ == '__main__':
    state_logger = StateLogger()
    logger.debug("kek")
    p = state_logger.read_all_data_for_drawing_froce_comp("sensor_1_touch_small_pike")
    p = state_logger.data_preprocessing_for_force_comparison(p)
    state_logger.draw_force_comparison(p)