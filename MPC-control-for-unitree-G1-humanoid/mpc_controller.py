import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


class ModelPredictiveController:

    def __init__(self, pred_h, cont_h, Q, R, R_d, P, sampling_time):
        self.pred_h = pred_h
        self.cont_h = cont_h
        self.Q = Q #state error cost
        self.R = R #control cost
        self.R_d = R_d #smooth control cost
        self.P = P # terminal state python
        self.sampling_time = sampling_time

    def cost(self,u, x, x_r):

        c = 0

        for i in range(self.pred_h):

            u_k = u[i*3:i+1*3]

            x_k = model(u_k,x)

            print(x_k.T.shape)
            print(x_r.shape)

            state_e_cost = (x_k-x_r) @ self.Q @ (x_k - x_r).T
            cont_cost = u_k.T @ self.R @ u_k

        return c

    def calc_constraints(self,map):
        pass

    def next_u(self,u,x, x_r, map, u_bounds):

        nu = len(u) # number of inputs
        u0 = np.zeros(self.pred_h * nu) #initial guess

        #state_constraints = calc_constraints(map)

        input_bounds = [u_bounds] * (self.pred_h * nu)

        res = minimize(self.cost,
                     u0,
                     args=(x,x_r),
                     method="SLSQP",
                     bounds = input_bounds)

        u_n = res.x[:nu]

        return u_n





def model(u, x):

    x_n = x + u

    return x_n


def main():

    T = []

    x = np.zeros(3)
    u = 1

    goal = np.array([1,1,1])

    Q = 10
    R = 10
    pred_h = 10
    cont_h = 10

    mpc = ModelPredictiveController(pred_h,cont_h,Q,R,0,0,0)

    u0 = np.zeros(3)

    u_bounds = (-0.1,0.1)
    while True:

        u = mpc.next_u(u0,x,goal,None,u_bounds)

        print(u)


        break



main()

