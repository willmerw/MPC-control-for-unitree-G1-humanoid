import matplotlib.pyplot as plt
import numpy as np
from ModelPredictiveController import *
from first_order_delay_model import *
import random
import math


def main():


    i = 0
    sim_len = 20



    goal = (10,10)



    obs = np.array([5,4])


    model = first_order_delay_model
    mpc = ModelPredictiveController(model)
    x = np.zeros(mpc.n_states)
    u = np.zeros(mpc.n_inputs)
    x_r = [goal]*mpc.pred_h



    u_bounds = (-1,1)

    T = [] #time

    X = [] #model x values

    Y = [] # model y values

    R = [] # reference



    while True:

        u = mpc.next_u(x,x_r,u_bounds)

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