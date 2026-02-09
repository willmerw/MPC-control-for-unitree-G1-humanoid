import matplotlib.pyplot as plt
import numpy as np
from ModelPredictiveController import *
import random
import math


def main():

    n_inputs = 2
    n_states = 2

    i = 0
    sim_len = 20

    x = np.zeros(n_states)
    u = np.zeros(n_inputs)

    Q = 10 # state cost
    R = 0.1 # control cost
    R_d = 1 # smoothness cost
    T = 100 # terminal state cost
    O = 200 # obstacle cost
    pred_h = 10
    cont_h = 10

    goal = (10,10)

    x_r = [goal]*pred_h

    obs = np.array([5,4])



    mpc = ModelPredictiveController(n_states,n_inputs,pred_h,cont_h,Q,R,R_d,T,O,0)



    u_bounds = (-1,1)

    T = [] #time

    X = [] #model x values

    Y = [] # model y values

    R = [] # reference



    while True:

        u = mpc.next_u(x,x_r,None,u_bounds)

        X.append(x[0])

        Y.append(x[1])

        x = model(u,x)

        T.append(i)
        i+=1


        if i > sim_len:
            break
    fig, ax = plt.subplots()
    plt.axis([-1,11,-1,11])
    plt.scatter(goal[0],goal[1],label="Goal")
    plt.plot(X,Y,label="MPC")
    circle1 =plt.Circle(obs, 1, color='r')
    circle2 =plt.Circle(obs, 2, color='green')
    ax.add_patch(circle2)
    ax.add_patch(circle1)
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.savefig("plot_gp.png")

main()