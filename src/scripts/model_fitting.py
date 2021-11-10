import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

from matplotlib import rcParams
rcParams['figure.figsize'] = (10, 6)
rcParams['legend.fontsize'] = 16
rcParams['axes.labelsize'] = 16


def fun_test(x, t):
    p = 100
    t_0 = 0

    k_p = x[1]*np.exp(-x[2]*p)
    tau_res = x[4]+x[5]*np.exp(-p/x[6])

    return x[0] + p*(k_p+x[3]*(1-np.exp(-(t-t_0)/tau_res)))*(1-np.exp(-x[7]/p))

def fun(x, t, y):
    p = 100
    t_0 = 0

    k_p = x[1]*np.exp(-x[2]*p)
    tau_res = x[4]+x[5]*np.exp(-p/x[6])

    return x[0] + p*(k_p+x[3]*(1-np.exp(-(t-t_0)/tau_res)))*(1-np.exp(-x[7]/p)) - y

def generate_data(x,t,noise=0,n_outliers=0,random_state=0):
    yy = fun_test(x,t)
    rnd = np.random.RandomState(random_state)
    error = noise * rnd.randn(t.size)
    outliers = rnd.randint(0, t.size, n_outliers)
    error[outliers] *= 35
    return yy + error

def check_general_algo():
    x_kek = [0.047,2.592,0.693,0.736,0.4,16.5,0.4,2.0]
    x_init = x_kek.copy()
    t_train = np.linspace(0,100,10000)
    y_train = generate_data(x_kek,t_train,noise=0.9,n_outliers=30)


    t_test = np.linspace(0,100,1000)
    y_test = generate_data(x_kek,t_test)
    res_robust = least_squares(fun, x_init, loss="soft_l1", f_scale= 0.1, args=(t_train, y_train))
    print(res_robust.x)
    y_robust = generate_data([*res_robust.x], t_test)



    plt.plot(t_train, y_train, 'o', label='data')
    plt.plot(t_test, y_test, label='true')
    plt.plot(t_test, y_robust, label='robust lsq')
    plt.xlabel('$t$')
    plt.ylabel('$y$')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    check_general_algo()