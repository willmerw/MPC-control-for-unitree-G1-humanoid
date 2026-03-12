import numpy as np
import math
import matplotlib.pyplot as plt

def first_order_delay_model(u,uf_prev,X_k):


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

    X = X_k.copy()

    x = X[0]
    v_x = X[1]
    y = X[2]
    v_y = X[3]
    z = X[4]
    v_z = X[5]

    u_f = u.copy()
    u_x = u_f[0]*np.cos(z) - u_f[1]*np.sin(z)
    u_y = u_f[0]*np.sin(z) + u_f[1]*np.cos(z)


    u_x = u_x
    u_y = u_y
    u_z = u_f[2]

    x_n = x + T*v_x
    y_n = y + T*v_y
    z_n = z + T*v_z

    v_x_n = math.exp(-T/tau_x)*v_x + (k_x-k_x*math.exp(-T/tau_x))*u_x
    v_y_n = math.exp(-T/tau_y)*v_y + (k_y-k_y*math.exp(-T/tau_y))*u_y
    v_z_n = math.exp(-T/tau_z)*v_z + (k_z-k_z*math.exp(-T/tau_z))*u_z

    #v_x_n = u_x
    #v_y_n = u_y
    #v_z_n = u_z

    x_out = np.array([x_n, y_n, z_n])

    X_n = np.array([x_n, v_x_n,y_n, v_y_n, z_n, v_z_n])

    return x_out, X_n,0

if __name__ == "__main__":
    u = np.array([2,0,1])
    uf_prev = np.zeros(3)

    X = np.zeros(6)
    X[4] = np.deg2rad(0) # set yaw to 90 degrees


    #  run it for 10 steps
    xpos, ypos, yawpos = [], [], []
    xV, yV, zV = [], [], []

    vx_out = []

    for i in range(60):
        x_out, X_n, _ = first_order_delay_model(u, uf_prev, X)
        X = X_n

        print("============ Step: {} ============".format(i))
        print("x_out:", np.array2string(x_out, precision=3, suppress_small=True, floatmode='fixed'))


        xpos.append(x_out[0])
        ypos.append(x_out[1])
        yawpos.append(x_out[2])

        xV.append(X_n[1])
        yV.append(X_n[3])
        zV.append(X_n[5])




