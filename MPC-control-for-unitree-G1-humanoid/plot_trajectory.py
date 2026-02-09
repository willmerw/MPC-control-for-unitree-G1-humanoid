import matplotlib.pyplot as plt
import numpy as np
from ModelPredictiveController import *
import random
import math


def gen_trajectory(length):

    traj_x = []
    traj_y = []
    traj = []

    for i in range(length):

        x = math.sin(i*0.5)
        y = i *0.5
        traj_x.append(x)
        traj_y.append(y)

        traj.append([x,y])

    return traj_x, traj_y,traj

def main():

    n_inputs = 2
    n_states = 2

    x = np.zeros(n_states)
    u = np.zeros(n_inputs)

    Q = 10 # state cost
    R = 3 # control cost
    R_d = 10 # smoothness cost
    T = 0.3 # terminal state cost
    pred_h = 10
    cont_h = 10

    traj_x,traj_y,traj = gen_trajectory(500)



    mpc = ModelPredictiveController(n_states,n_inputs,pred_h,cont_h,Q,R,R_d,T,0)



    u_bounds = (-1,1)

    T = [] #time

    X = [] #model x values

    Y = [] # model y values

    R = [] # reference

    i = 0
    sim_len = 20

    while True:

        x_r = traj[i:i+pred_h]

        u = mpc.next_u(x,x_r,None,u_bounds)

        X.append(x[0])

        Y.append(x[1])

        x = model(u,x)




        #R.append(traj[i][0])

        T.append(i)
        i+=1


        if i > sim_len:
            break

    plt.axis([-2,2,0,20])
    plt.scatter(traj_x,traj_y, label="Trajectory")
    plt.plot(X,Y,label="MPC")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.savefig("plot.png")

main()


