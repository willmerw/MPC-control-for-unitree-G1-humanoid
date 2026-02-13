import numpy as np
import matplotlib.pyplot as plt
import math

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

cmd_vels = np.load("g1data/cmd_vels.npy")
g2_accels = np.load("g1data/g2_accels.npy")
g2_twist = np.load("g1data/g2_twist.npy")




cmd_x_range = range(0,7)
cmd_vel_times = cmd_vels[cmd_x_range,0]
cmd_vel_x = cmd_vels[cmd_x_range,1]


x_range = range(900,2500)
g2_twist_times_x = g2_twist[x_range,0]
g2_twist_x = g2_twist[x_range,1]


m = 100
g2_twist_x_m = g2_twist_x.copy()

# data filtering

for i in range(len(g2_twist_x_m)):
    if g2_twist_x_m[i] > 1.2:
        g2_twist_x_m[i] = 1
    elif g2_twist_x_m[i] < 0 and i > 1:
        g2_twist_x_m[i] = g2_twist_x_m[i-1]

for i in range(m,len(g2_twist_x)-m):
    g2_twist_x_m[i] = min(1,np.mean(g2_twist_x_m[i-m:i+m]))



x = np.zeros((2,1))
u_f = 0
time = []
X = []
U = []

tau = 0.1 # time constant
tau_u = 0.7
T= 0.01 # sampling time
k = 0.85 #DC gain

for i in np.arange(g2_twist_times_x[0],g2_twist_times_x[-1],T):
    if i > cmd_vel_times[0] :
        U.append(cmd_vel_x[0])
    else:
        U.append(0)
    time.append(i)


for i in range(len(time)):
    u = U[i]
    x,u_f = first_order_delay_model(u,u_f,x,tau,tau_u,T,k)
    X.append(x[1])

plt.figure()
plt.clf()
plt.axis([time[0],time[-1],0,1])
plt.plot(time,X,label="Model response")
#plt.plot(g2_twist_times_x,g2_twist_x)
#plt.plot(g2_twist_times_x,g2_twist_x,label="G1 X velocity")
plt.plot(g2_twist_times_x[:-m],g2_twist_x_m[:-m],label="Filtered G1 X velocity, m = 100")
#plt.scatter(g2_twist_times,g2_twist_z)
plt.plot(cmd_vel_times,cmd_vel_x,label="Cmd X velocity")
#plt.scatter(g2_accel_times,g2_accel_x)
#plt.plot(unit1_accels_times,unit1_accels_x)
#plt.plot(unit3_accels_times,unit3_accels_x)

plt.legend()

plt.savefig("plot.png")