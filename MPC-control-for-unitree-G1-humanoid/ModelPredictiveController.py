import numpy as np
from scipy.optimize import minimize
import math


class ModelPredictiveController:

    def __init__(self,model):
        self.model = model
        self.pred_h = 20
        self.cont_h = 15

        self.Q = np.diag([1,1]) * 10 #state error cost

        self.Q_yaw = 0

        self.R = np.diag([1,5,0]) * 1  #control cost

        self.R_d = np.diag([1,1,1])* 1 #smooth control cost

        self.T = np.diag([1,1]) * 0 # terminal state cost

        self.O = 0.7 # obstacle avoidance

        self.sampling_time = 0.1

        self.n_states = 3
        self.n_inputs = 3

        self.x_v_min = 0
        self.x_v_max = 0.3

        self.y_v_min = -0.01
        self.y_v_max = 0.01

        self.z_v_min = -0.5
        self.z_v_max = 0.5

        self.obs_r = 0.5
        self.obs_r_inf = 1

        self.pts = [[0,0]]

    def cost(self,u, x, x_r):

        c = 0

        x_n = x.copy()

        u_k_prev = np.zeros(self.n_inputs)
        u_prev = np.zeros(self.n_inputs)

        for i in range(self.pred_h):

            if i < self.cont_h:
                u_k = u[i*self.n_inputs:(i+1)*self.n_inputs]
            else:
                u_k = u[(self.cont_h-1)*self.n_inputs:self.cont_h*self.n_inputs]

            x_r_k = x_r[i]

            x_out,x_n,u_prev = self.model(u_k,u_prev,x_n)

            x_y = x_out[:2]
            yaw_k = x_out[2]

            #obstacle cost

            #c += self.obstacle_cost(x_out) * self.O

            #smoothness cost
            du = u_k-u_k_prev

            c += du @ self.R_d @ du

            u_k_prev = u_k

            #position error cost
            e = x_y-x_r_k

            pos_e_cost = e @ self.Q @ e

            c += pos_e_cost

            #yaw error cost

            e = x_r_k-x_y
            dx = e[0]
            dy = e[1]

            yaw_r = math.atan2(dy,dx)

            y_e = yaw_r - yaw_k

            yaw_cost = y_e**2 * self.Q_yaw

            c += yaw_cost


            #control input cost
            cont_cost = u_k @ self.R @ u_k

            c += cont_cost


        #terminal state cost
        e_T = x_y - x_r[-1]
        terminal_cost = e_T @ self.T @ e_T
        c += terminal_cost
        return c

    def obstacle_cost(self, x_out):

        c = 0

        for pt in self.pts:

            dx = pt[0] - x_out[0]
            dy = pt[1] - x_out[1]

            d = np.sqrt(dx**2 + dy**2)

            if d > self.obs_r_inf:
                c += 0
            else:
                c += (1/((abs(self.obs_r-d)+1e-6)))**2
        return c

    def next_u(self,x, x_r, u_prev):

        u0 = np.zeros(self.cont_h * self.n_inputs)

        #u0 = u_prev.copy()

        input_bounds = [
            [self.x_v_min, self.x_v_max],
            [self.y_v_min, self.y_v_max],
            [self.z_v_min, self.z_v_max]
            ] * self.cont_h

        res = minimize(self.cost,
                     u0,
                     args=(x,x_r),
                     method="SLSQP",
                     bounds = input_bounds
                     )

        u_n = res.x[:self.n_inputs]

        u_opt = res.x.reshape(self.cont_h, self.n_inputs)


        # --- Forward simulate predicted states ---
        x_pred = np.zeros((self.cont_h + 1, self.n_states))


        x_n = x.copy()

        for k in range(self.cont_h):
            x_pred[k],x_n,u_prev = self.model(u_opt[k],u_prev,x_n)

        return u_n,x_pred[:len(x_pred)-1],res.x




