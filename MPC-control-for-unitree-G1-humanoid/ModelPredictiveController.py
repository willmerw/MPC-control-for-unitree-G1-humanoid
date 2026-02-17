import numpy as np
from scipy.optimize import minimize
import math


class ModelPredictiveController:

    def __init__(self,model,n_states,n_inputs, pred_h, cont_h, Q, R, R_d, T,O, sampling_time):
        self.model = model
        self.pred_h = pred_h
        self.cont_h = cont_h
        self.Q = np.eye(n_states) *Q #state error cost
        self.R = np.eye(n_inputs) * R #control cost
        self.R_d = np.eye(n_inputs)* R_d #smooth control cost
        self.T = np.eye(n_states) * T # terminal state cost
        self.O = O # obstacle avoidance
        self.sampling_time = sampling_time
        self.n_states = n_states
        self.n_inputs = n_inputs

    def cost(self,u, x, x_r,const_f=None):

        c = 0

        x_k = x.copy()

        u_k_prev = np.zeros(self.n_inputs)

        for i in range(self.pred_h):



            u_k = u[i*self.n_inputs:(i+1)*self.n_inputs]

            x_r_k = x_r[i]

            x_k = self.model(u_k,x_k)

            #obstacle cost

            c += self.obstacle_cost(x_k) * self.O



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

    def calc_constraints(self,f,args):
        constraints = {
            "type": "ineq",
            "fun": f,
            "args": args
        }
        return constraints

    def obstacle_cost(self,x_n):
        c_obs = np.array([5.0, 4.0])
        r = 2.0

        dx = x_n[0] - c_obs[0]
        dy = x_n[1] - c_obs[1]

        g = dx*dx + dy*dy - (r)**2

        return max(0.0, g)**2




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




