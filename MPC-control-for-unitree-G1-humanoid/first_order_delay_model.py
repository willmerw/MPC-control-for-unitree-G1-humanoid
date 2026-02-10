import numpy as np
import math
import matplotlib.pyplot as plt

def first_order_delay_model(u,x,tau,T):
    v = x[:2]
    x_k = x[2:]

    x_k_n = x_k + v

    v_n = math.exp(-T/tau)*v + (1- math.exp(-T/tau))*u

    x_n = np.concatenate((v_n,x_k_n))
    return x_n

x = np.zeros(4)
u = np.ones(2)

tau = 1
T = 0.1

X = []
T = []

for i in range(100):
    x = first_order_delay_model(u,x,tau,T)

    X.append(x[0])
    T.append(i)


plt.plot(T,X)
plt.savefig("model.png")
