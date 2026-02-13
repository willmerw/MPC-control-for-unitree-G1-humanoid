import numpy as np
import math
import matplotlib.pyplot as plt

def first_order_delay_model(u,uf_prev,x,tau,tau_u,T,k):


    A = np.array([
    [0.0, 1.0],
    [0.0, math.exp(-T/tau)],
    ])

    B = np.array([
        [0.0],
        [k-k*math.exp(-T/tau)]
    ])

    u_f = math.exp(-T/tau_u) * uf_prev + (1 - math.exp(-T/tau_u)) * u

    x_n = A@x + B*u_f

    return x_n, u_f

"""
x = np.zeros((2,1))
u = 1
u_f = 0


time = []
X = []
tau = 0.01
tau_u = 10
T = 0.1
k = 0.8


for i in range(1000):
    x,u_f = first_order_delay_model(u,u_f,x,tau,tau_u,T,k)
    time.append(i)
    X.append(x[0])
    print(x)

plt.plot(time,X)
plt.savefig("model.png")
"""