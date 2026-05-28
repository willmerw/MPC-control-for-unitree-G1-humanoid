from matplotlib.patches import Circle
import opengen as og
from first_order_delay_model import *
import time
# Create a TCP connection manager

mng = og.tcp.OptimizerTcpManager("mpc_build/optimized_mpc")

# Start the TCP server
mng.start()
print("Starting TCP server...")
pred_h=100
# Run simulations
x_state_0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Initial state: [x, v_x, y, v_y, z, v_z]
ref = [5.0, 5.0] *pred_h
obs = [1.5, 1.5, 3.5, 2.0] * pred_h
simulation_steps = 2000

state_sequence = x_state_0
input_sequence = []

x_nr = x_state_0 + ref + obs
initial_guess = None
current_time = time.time()
xs = []
T = []
for k in range(simulation_steps):

    solver_status = mng.call(x_nr, initial_guess)
    us = solver_status['solution']
    u = us[0:3]
    #initial_guess = us[3:6] + [us]

    x_out,x_next= first_order_delay_model(u,x_state_0)
    xs.append(x_out)
    print(x_out)
    state_sequence += x_next
    input_sequence += [u]
    x_nr = np.concatenate((x_next, ref, obs))
    x_state_0 = x_next

# Thanks TCP server; we won't be needing you any more

mng.kill()

fig, ax = plt.subplots()

# Correct instantiation: define the circle patch
obstacle_circle = Circle((1.5, 1.5), 1, color='r', alpha=0.5, label='Obstacle')
obstacle_circle2 = Circle((3.5, 2), 1, color='r', alpha=0.5, label='Obstacle')
# Add the patch to the axis
ax.add_patch(obstacle_circle)
ax.add_patch(obstacle_circle2)
xs = np.array(xs)
from matplotlib.patches import Circle

plt.plot(xs[:,0], xs[:,1], label='Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('MPC Trajectory')
plt.legend()
plt.grid(True)
plt.show()