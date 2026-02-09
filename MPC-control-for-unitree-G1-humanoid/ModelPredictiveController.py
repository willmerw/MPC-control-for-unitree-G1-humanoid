import numpy as np
from scipy.optimize import minimize



class ModelPredictiveController:

    def __init__(self,n_states,n_inputs, pred_h, cont_h, Q, R, R_d, T, sampling_time):
        self.pred_h = pred_h
        self.cont_h = cont_h
        self.Q = np.eye(n_states) *Q #state error cost
        self.R = np.eye(n_inputs) * R #control cost
        self.R_d = np.eye(n_inputs)* R_d #smooth control cost
        self.T = np.eye(n_states) * T # terminal state cost
        self.sampling_time = sampling_time
        self.n_states = n_states
        self.n_inputs = n_inputs

    def cost(self,u, x, x_r):

        c = 0

        x_k = x.copy()

        u_k_prev = np.zeros(self.n_inputs)

        for i in range(self.pred_h):



            u_k = u[i*self.n_inputs:(i+1)*self.n_inputs]

            x_r_k = x_r[i]

            x_k = model(u_k,x_k)

            #smoothness cost
            du = u_k-u_k_prev

            c += du @ self.R_d @ du

            u_k_prev = u_k

            #state error cost
            e = x_k-x_r_k
            state_e_cost = e @ self.Q @ e

            c += state_e_cost

            #control input cost
            cont_cost = u_k @ self.R @ u_k

            c += cont_cost

            #terminal state cost
            e_T = x_k - x_r[-1]
            terminal_cost = e_T @ self.T @ e_T
            c += terminal_cost


        return c

    def calc_constraints(self,map):
        pass

    def next_u(self,x, x_r, map, u_bounds):

        u0 = np.zeros(self.pred_h * self.n_states) #initial guess

        #state_constraints = calc_constraints(map)

        input_bounds = [u_bounds] * (self.pred_h * self.n_states)

        res = minimize(self.cost,
                     u0,
                     args=(x,x_r),
                     method="SLSQP",
                     bounds = input_bounds)

        u_n = res.x[:self.n_states]

        return u_n


def model(u, x):

    x_n = x + u

    return x_n



