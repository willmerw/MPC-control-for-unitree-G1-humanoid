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


print(cmd_vels)
cmd_x_range = range(0,7)
cmd_vel_times_x = cmd_vels[cmd_x_range,0]
cmd_vel_x = cmd_vels[cmd_x_range,1]


x_range = range(900,2500)
g2_twist_times_x = g2_twist[x_range,0]
g2_twist_x = g2_twist[x_range,1]

cmd_y_range = range(7,14)
cmd_vel_times_y = cmd_vels[cmd_y_range,0]
cmd_vel_y = cmd_vels[cmd_y_range,2]


y_range = range(0,len(g2_twist))
g2_twist_times_y = g2_twist[y_range,0]
g2_twist_y = g2_twist[y_range,2
                      ]


m = 100
g2_twist_x_m = g2_twist_x.copy()

# data filtering

def moving_mean_filter(data,m,upper):
    for i in range(len(data)):
        if data[i] > upper*1.2:
            data[i] = upper
        elif data[i] < 0 and i > 1:
            data[i] = data[i-1]

    for i in range(m,len(data)-m):
        data[i] = min(1,np.mean(data[i-m:i+m]))

    return data

#Prepare data for model input
def fix_cmd(time_data,cmd_times,cmds,T,U,time):
    for i in np.arange(time_data[0],time_data[-1],T):
        if i > cmd_times[0] :
            U.append(cmds[0])
        else:
            U.append(0)
        time.append(i)

    return U, time

g2_twist_x_m = moving_mean_filter(g2_twist_x_m,m,1)


tau = 0.1 # time constant
tau_u = 0.7
T= 0.01 # sampling time
k = 0.85 #DC gain

x = np.zeros((2,1))
u_f = 0
time = []
X = []
U = []

U, time = fix_cmd(g2_twist_times_y,cmd_vel_times_y,cmd_vel_y,T,U,time)






for i in range(len(time)):
    u = U[i]
    x,u_f = first_order_delay_model(u,u_f,x,tau,tau_u,T,k)
    X.append(x[1])

plt.figure()
plt.clf()
plt.axis([time[0],time[-1],-1,1])

#X plotting
#plt.plot(time,X,label="Model response")
#plt.plot(g2_twist_times_x[:-m],g2_twist_x_m[:-m],label="Filtered G1 X velocity, m = 100")
#plt.plot(cmd_vel_times,cmd_vel_x,label="Cmd X velocity")
#plt.plot(g2_twist_times_x,g2_twist_x,label="G1 X velocity")

#Y plotting
#plt.plot(time,Y,label="Model response")
#plt.plot(g2_twist_times_y[:-m],g2_twist_y_m[:-m],label="Filtered G1 X velocity, m = 100")
plt.plot(cmd_vel_times_y,cmd_vel_y,label="Cmd Y velocity")
plt.plot(g2_twist_times_y,g2_twist_y,label="G1 Y velocity")


plt.legend()

plt.savefig("plot.png")