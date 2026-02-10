import numpy as np
import matplotlib.pyplot as plt
cmd_vels = np.load("g1data/cmd_vels.npy")
g2_accels = np.load("g1data/g2_accels.npy")
g2_twist = np.load("g1data/g2_twist.npy")




cmd_x_range = range(0,7)
cmd_vel_times = cmd_vels[cmd_x_range,0]
cmd_vel_x = cmd_vels[cmd_x_range,1]


x_range = range(900,2500)
g2_twist_times_x = g2_twist[x_range,0]
g2_twist_x = g2_twist[x_range,1]

m = 20
g2_twist_x_m = g2_twist_x.copy()

for i in range(len(g2_twist_x_m)):
    if g2_twist_x_m[i] > 1.2:
        g2_twist_x_m[i] = 1
    elif g2_twist_x_m[i] < 0 and i > 1:
        g2_twist_x_m[i] = g2_twist_x_m[i-1]

for i in range(m,len(g2_twist_x)-m):
    g2_twist_x_m[i] = min(1,np.mean(g2_twist_x_m[i-m:i+m]))



plt.plot(g2_twist_times_x[:-m],g2_twist_x_m[:-m],label="Filtered G1 X velocity, m = 20")

m = 100
g2_twist_x_m = g2_twist_x.copy()

for i in range(len(g2_twist_x_m)):
    if g2_twist_x_m[i] > 1.2:
        g2_twist_x_m[i] = 1
    elif g2_twist_x_m[i] < 0 and i > 1:
        g2_twist_x_m[i] = g2_twist_x_m[i-1]

for i in range(m,len(g2_twist_x)-m):
    g2_twist_x_m[i] = min(1,np.mean(g2_twist_x_m[i-m:i+m]))



#plt.axis([0,70,-2,2])

#plt.scatter(g2_twist_times,g2_twist_x)
#plt.plot(g2_twist_times_x,g2_twist_x,label="G1 X velocity")
plt.plot(g2_twist_times_x[:-m],g2_twist_x_m[:-m],label="Filtered G1 X velocity, m = 100")
#plt.scatter(g2_twist_times,g2_twist_z)
plt.plot(cmd_vel_times,cmd_vel_x,label="Cmd X velocity")
#plt.scatter(g2_accel_times,g2_accel_x)
#plt.plot(unit1_accels_times,unit1_accels_x)
#plt.plot(unit3_accels_times,unit3_accels_x)

plt.legend()

plt.savefig("plot.png")