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

        x = math.sin(i)
        y = i
        traj_x.append(x)
        traj_y.append(y)

        traj.append([x,y])

    return traj_x, traj_y,traj

def main():

    T = []

    x = np.zeros(2)

    Q = 1
    R = 2
    pred_h = 10
    cont_h = 10

    traj_x,traj_y,traj = gen_trajectory(50)

    mpc = ModelPredictiveController(len(x),pred_h,cont_h,Q,R,0,0,0)

    u = np.zeros(2)

    u_bounds = (-0.1,0.1)

    X = [] #model x values

    Y = [] # model y values

    R = [] # reference

    i = 0
    sim_len = 100

    while True:

        u = mpc.next_u(u,x,traj,None,u_bounds)

        x = model(u,x)


        X.append(x[0])

        Y.append(x[1])

        #R.append(goal_point[0])

        T.append(i)
        i+=1


        if i > sim_len:
            break

    plt.axis([-2,2,0,4])

    plt.scatter(traj_x,traj_y, label="Trajectory")
    plt.plot(X,Y,label="MPC")
    plt.legend()
    plt.savefig("plot.png")

main()


