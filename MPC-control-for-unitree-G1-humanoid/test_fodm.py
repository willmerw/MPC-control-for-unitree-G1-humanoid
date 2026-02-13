from first_order_delay_model import *
import ModelPredictiveController as MPC

n_inputs = 2
n_states = 2

x = np.zeros(n_states)
u = np.zeros(n_inputs)

Q = 10 # state cost
R = 3 # control cost
R_d = 10 # smoothness cost
T = 0.3 # terminal state cost
pred_h = 10
cont_h = 10

traj_x,traj_y,traj = gen_trajectory(500)



mpc = MPC(n_states,n_inputs,pred_h,cont_h,Q,R,R_d,T,0)

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