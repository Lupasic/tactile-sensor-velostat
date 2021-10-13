    def draw_plots(self, x_act, x_des, time, force_data):
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