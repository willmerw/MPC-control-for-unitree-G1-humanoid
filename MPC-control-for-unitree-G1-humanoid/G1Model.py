import numpy as np


class G1Model:

    def __init__(self, n_states, n_inputs):
        self.n_states = n_states
        self.n_inputs =  n_inputs

        self.x = np.zeros(n_states)

    def update(self,u):

        self.x += u

        return self.x