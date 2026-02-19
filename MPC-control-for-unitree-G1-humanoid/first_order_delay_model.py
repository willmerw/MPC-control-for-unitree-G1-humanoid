import numpy as np
import math
import matplotlib.pyplot as plt

def first_order_delay_model(u,uf_prev,x):

    """
    x = [x x_dot y y_dot,z,z_dot].T
    """
    T = 0.1
    tau_x = 0.1 # time constant x
    tau_u_x = 0.7 # input time constant x
    k_x = 0.85 #DC gain x

    tau_y = 0.1 # time constant x
    tau_u_y = 0.7 # input time constant x
    k_y = 0.85 #DC gain x

    tau_z = 0.1 # time constant x
    tau_u_z = 0.7 # input time constant x
    k_z = 0.85 #DC gain x

    A = np.array([
    [1.0, T, 0.0, 0.0, 0.0, 0.0],
    [0.0, math.exp(-T/tau_x), 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, T, 0.0, 0.0],
    [0.0, 0.0, 0.0, math.exp(-T/tau_y), 0.0, 0.0 ],
    [0.0, 0.0, 0.0, 0.0, 1.0, T],
    [0.0, 0.0, 0.0, 0.0, 0.0, math.exp(-T/tau_z)]
    ])

    B = np.array([
        [0.0, 0.0, 0.0],
        [k_x-k_x*math.exp(-T/tau_x), 0.0, 0.0],
         [0.0, 0.0, 0.0],
         [0.0, k_y-k_y*math.exp(-T/tau_y), 0.0],
         [0.0, 0.0, 0.0],
         [0.0, 0.0, k_z-k_z*math.exp(-T/tau_z)]
    ])

    u_delay = np.array([
        [math.exp(-T/tau_u_x), 0.0],
        [0.0, math.exp(-T/tau_u_y)],
        [0.0, math.exp(-T/tau_u_z)]
    ])

    C = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]
                  ])

    #u_f = u_delay @ uf_prev + (np.eye(len(u)) - u_delay) @ u

    u_f = u


    x_n = A@x + B@u_f

    y = C@x


    return y, x_n, u_f

"""
x = np.zeros((2,1))
u = 1
u_f = 0


time = []
X = []
tau = 0.01
tau_u = 10
T = 0.1
k = 0.8


for i in range(1000):
    x,u_f = first_order_delay_model(u,u_f,x,tau,tau_u,T,k)
    time.append(i)
    X.append(x[0])
    print(x)

plt.plot(time,X)
plt.savefig("model.png")
"""