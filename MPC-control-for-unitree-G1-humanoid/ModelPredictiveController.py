import numpy as np
from scipy.optimize import minimize



class ModelPredictiveController:

    def __init__(self,n_states, pred_h, cont_h, Q, R, R_d, P, sampling_time):
        self.pred_h = pred_h
        self.cont_h = cont_h
        self.Q = np.eye(n_states) *Q #state error cost
        self.R = np.eye(n_states) * R #control cost
        self.R_d = R_d #smooth control cost
        self.P = P # terminal state python
        self.sampling_time = sampling_time
        self.n_states = n_states

    def cost(self,u, x, x_r):

        c = 0



        for i in range(self.pred_h):

            u_k = u[i*self.n_states:(i+1)*self.n_states]

            x_r_k = x_r[i*self.n_states:(i+1)*self.n_states]

            x_k = model(u_k,x)

            e = x_r_k-x_k
            state_e_cost = e @ self.Q @ e

            c += state_e_cost

            u_k = u_k.reshape(-1,1)
            cont_cost = u_k.T @ self.R @ u_k

            c += cont_cost



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





