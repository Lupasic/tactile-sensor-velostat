import urx
from time import perf_counter
from numpy import array, frexp, sin, cos, pi, linspace, random
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from multiprocessing import Process, Value
from futek_sensor import FutekSensor

vir_stiff = 500
freq_mod_traj = 100


class urxrobotPlus(urx.Robot):
    def __init__(self, address):
        super().__init__(address,use_rt=False,use_simulation=False)

    # def get_speed(self):
    #     self.send_program("get_actual_tcp_speed")


def f_x(delta_prev,t,F_cur):
    global vir_stiff
    return F_cur - vir_stiff*delta_prev


def solve_diff_eq(freq_solve, delta_prev, F_cur):
    t0 = perf_counter()
    t = linspace(0,1/freq_solve,5)
    delta = odeint(f_x, delta_prev,t, args = (F_cur,))[-1][0]
    ddelta = F_cur - vir_stiff * delta
    ttt = perf_counter() - t0
    return delta, ddelta, ttt
    

def modify_z_g(F_d, z_g):
    global vir_stiff
    return z_g - F_d/vir_stiff

def traj_mod_based_on_force(z_g_mod, z_prev, F_cur, dz_g=0):
    global freq_mod_traj
    delta_prev = z_g_mod - z_prev
    delta, ddelta, __ = solve_diff_eq(freq_mod_traj, delta_prev, F_cur)
    z_d_mod = z_g + delta
    dz_d_mod = dz_g + ddelta
    return z_d_mod, dz_d_mod





def manipulator_control(rob):

    k = 5
    A = 0.1
    freq = 0.2
    omega = 2*pi*freq
    # A0 = 0.5
    A0 = rob.getl()[2]
    z_act = []
    z_des = []
    time = []
    freq_log  = 100
    try: 
        t0 = perf_counter()
        t1 = 0
        while True: 
            t = perf_counter() -t0
            state = rob.getl()

            z_d = A0 + A*sin(omega*t)
            dz_d = A*omega*cos(omega*t)
            x ,y ,z = array(state[:3])
            u_z = dz_d + k*(z_d - z)

            if t - t1 >1/freq_log:
        
                z_act.append(z)
                z_des.append(z_d)
                time.append(t)
                t1 = t

            rob.speedx("speedl",[0,0,u_z,0,0,0], 0.5, 5)
            
            print(f'{z.round(3)} {z_d.round(3)} {z_d.round(3) - z.round(3)}', end = '\r', flush = True)

    except KeyboardInterrupt:
        rob.speedx("speedl",[0,0,0,0,0,0], 0.5, 1)
        print('Robot is stopped')

        plt.plot(time, array(z_act), label="actual")
        plt.plot(time, array(z_des), label="needed")
        plt.plot(time, array(z_act) - array(z_des), label="error")
        plt.title("Manipulator control, k = " + str(k))
        plt.xlabel("time, sec")
        plt.ylabel("position, m")
        plt.legend(loc="upper right")
        plt.show()

def check_ode_int_time_consumption():
    t = []
    for i in range(1000):
        ini = random.uniform(0, 1)
        ff = random.uniform(0, 10)
        delta, ddelta, ttt = solve_diff_eq(100, ini, ff)
        t.append(ttt)
    print(sum(t) / len(t))

def read_data_from_force_sensor(futek, received_data):
    while True:
        rec_val =  futek.raw2F(futek.readData(write_to_file=1))
        if rec_val > 1:
            received_data.value = rec_val
        else:
            received_data.value = 0

if __name__ == '__main__':
    address="172.31.1.25"
    rob = urxrobotPlus(address)
    print(rob)
    # rob.speedx("speedl",[0,0,0.0,0,0,0], 0.5, 5)
    # manipulator_control(rob)
    # check_ode_int_time_consumption()
    received_data = Value('d', 0.0)
    futek = FutekSensor(port='/dev/ttyUSB0',file_name="futek_testing_ur", debug=0)
    p = Process(target=read_data_from_force_sensor, args=(futek, received_data))
    p.start()
    # try:
    #     F_d = 5
    #     z_g = rob.getl()[2]
    #     # z_g = modify_z_g(F_d,z_g)
    #     z_prev = rob.getl()[2]
    #     while True:
    #         z_mod, dz_mod = traj_mod_based_on_force(z_g, z_prev, received_data.value, dz_g=0)
    #         print(f'{z_g} {z_mod}', end = '\r', flush = True)
    # except KeyboardInterrupt:
    #     p.terminate()


    k = 5
    z_act = []
    z_des = []
    time = []
    freq_log  = 100
    F_d = 5
    z_g = rob.getl()[2]
    z_g = modify_z_g(F_d,z_g)
    # z_g = 0.4
    z_prev = rob.getl()[2]
    try: 
        t0 = perf_counter()
        t1 = 0
        while True: 
            t = perf_counter() -t0
            state = rob.getl()

            z_mod, dz_mod = traj_mod_based_on_force(z_g, z_prev, received_data.value, dz_g=0)
            z_d = z_mod
            dz_d = dz_mod
            z_prev = z_mod
            x ,y ,z = array(state[:3])
            u_z = dz_d + k*(z_d - z)

            if t - t1 >1/freq_log:
        
                z_act.append(z)
                z_des.append(z_d)
                time.append(t)
                t1 = t

            rob.speedx("speedl",[0,0,u_z,0,0,0], 0.5, 5)
            print(f'{z.round(3)} {z_d.round(3)} {z_d.round(3) - z.round(3)}', end = '\r', flush = True)
    except KeyboardInterrupt:
        p.terminate()
        rob.speedx("speedl",[0,0,0,0,0,0], 0.5, 1)
        print('Robot is stopped')

        plt.plot(time, array(z_act), label="actual")
        plt.plot(time, array(z_des), label="needed")
        plt.plot(time, array(z_act) - array(z_des), label="error")
        plt.title("Manipulator control, k = " + str(k))
        plt.xlabel("time, sec")
        plt.ylabel("position, m")
        plt.legend(loc="upper right")
        plt.show()



    # try:
    #     while True:
    #         print(received_data.value, end = '\r', flush = True)
    # except KeyboardInterrupt:
    #     p.terminate()
    #     print("end")