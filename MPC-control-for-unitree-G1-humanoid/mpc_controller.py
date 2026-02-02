import numpy as np

class ModelPredictiveController:

    def __init__(self, pred_h, cont_h, Q, R, R_d, P, sampling_time):
        self.pred_h = pred_h
        self.cont_h = cont_h
        self.Q = Q
        self.R = R
        self.R_d = R_d
        self.P = P
        self.sampling_time = sampling_time

    def cost(u, x, traj):

        c = 0
        for i in range(self.pred_h):
            x_k = model(u)

            x_r = traj[i]

            state_e_cost = (x_k-x_r).T @ self.Q @ (x_k - x_r)
            cont_cost = u.T @ self.R @ u



def model(u):

    return u

