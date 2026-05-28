import os
import casadi as cs
import opengen as og

# ==============================================================================
# 1. CASADI-COMPATIBLE SYSTEM MODEL
# ==============================================================================
def first_order_delay_model_casadi(u, X_k):
    T = 0.1
    # Model parameters
    tau_x, tau_u_x, k_x = 1.0, 1.9, 0.55
    tau_y, tau_u_y, k_y = 1.0, 2.2, 0.55
    tau_z, tau_u_z, k_z = 1.0, 2.2, 0.85

    # Unpack symbolic states
    x   = X_k[0]
    v_x = X_k[1]
    y   = X_k[2]
    v_y = X_k[3]
    z   = X_k[4]
    v_z = X_k[5]

    # Global frame velocity conversions using CasADi trig functions
    u_x = u[0] * cs.cos(z) - u[1] * cs.sin(z)
    u_y = u[0] * cs.sin(z) + u[1] * cs.cos(z)
    u_z = u[2]


    # Kinematic updates
    x_n = x + T * v_x
    y_n = y + T * v_y
    z_n = z + T * v_z

    # Dynamic velocity updates with exponential delay mechanics
    v_x_n = cs.exp(-T / tau_x) * v_x + (k_x - k_x * cs.exp(-T / tau_u_x)) * u_x
    v_y_n = cs.exp(-T / tau_y) * v_y + (k_y - k_y * cs.exp(-T / tau_u_y)) * u_y
    v_z_n = cs.exp(-T / tau_z) * v_z + (k_z - k_z * cs.exp(-T / tau_u_z)) * u_z

    # Pack back into CasADi structural vectors
    x_out = cs.vertcat(x_n, y_n, z_n)
    X_n   = cs.vertcat(x_n, v_x_n, y_n, v_y_n, z_n, v_z_n)

    return x_out, X_n


# ==============================================================================
# 2. MPC PROBLEM CONFIGURATION & SYMBOLIC GRAPH
# ==============================================================================
# Problem Dimensions
pred_h = 50
cont_h = 50
n_states = 6
n_inputs = 3
n_obstacles = 2 # Number of obstacles to track

# Define Decision Variables (The control input sequence)
u_sequence = cs.SX.sym('u_seq', n_inputs * cont_h)

# Define Parameters (Current 6-state vector + reference points x,y for each prediction step)
x_init = cs.SX.sym('x_init', n_states)
x_ref  = cs.SX.sym('x_ref', 2 * pred_h)
if n_obstacles > 0:
    obs_ref   = cs.SX.sym('obs_ref', 2 * pred_h * n_obstacles)
    params = cs.vertcat(x_init, x_ref, obs_ref)
else:
    params = cs.vertcat(x_init, x_ref)

# Cost and State Initialization
cost = 0
X_n = x_init
u_k_prev = cs.DM.zeros(n_inputs)

obs_r = 0.5
obs_r_inf = 1.0
O_weight = 100.0
decay_rate=1.0
# Penalty Matrices (Constructed using CasADi structural matrices)
Q   = cs.diag([10, 10])     # State error weight
R   = cs.diag([1, 50, 0])    # Control input cost weight
R_d = cs.diag([1, 1, 1])    # Input rate-of-change smoothness weight
T_m = cs.diag([0.1, 0.1])   # Terminal weight

if "__main__" == __name__:
# Symbolic Prediction Loop
    for i in range(pred_h):
        # Slice target control sequence based on control horizon limits
        u_idx = min(i, cont_h - 1) * n_inputs
        u_k = u_sequence[u_idx : u_idx + n_inputs]

        # Extract reference coordinates for current step
        x_r_k = x_ref[i * 2 : (i + 1) * 2]

        # Evaluate model forward step symbolically
        x_out, X_n = first_order_delay_model_casadi(u_k, X_n)

        # Extract calculated spatial positions
        x_y = x_out[0:2]

        # Position tracking error evaluation
        e_pos = x_y - x_r_k
        cost += cs.bilin(Q, e_pos, e_pos)

        # Smoothness evaluation (delta-u penalty)
        du = u_k - u_k_prev
        cost += cs.bilin(R_d, du, du)
        u_k_prev = u_k

        # Control amplitude penalty
        cost += cs.bilin(R, u_k, u_k)

        # Obstacle Proximity Cost (Continuous symbolic barrier function)
        for m in range(n_obstacles):


            base_idx = (m * 2) + (i * 2)*n_obstacles #CURRENTLY ASSUMES STATIC OBSTACLES

            #base_idx = (m * 2 *pred_h) + (i * 2) # Dynamic obstacles indexing (uncomment if using dynamic obstacles) (requires obs_ref to be sized for dynamic obstacles)

            obs_x = obs_ref[base_idx + 0]
            obs_y = obs_ref[base_idx + 1]

            dx = obs_x - x_out[0]
            dy = obs_y - x_out[1]
            d = cs.sqrt(dx**2 + dy**2)

            barrier = cs.exp(-decay_rate * (d - obs_r))

            # Smooth conditional window using cs.if_else
            barrier_windowed = cs.if_else(d <= obs_r_inf, barrier, 0.0)

            cost += O_weight * barrier_windowed

            #cost += O_weight * cs.fmax(0, (1.0 / (cs.fabs(d-obs_r) + 1e-6)) - (1.0 / (cs.fabs(obs_r_inf-obs_r) + 1e-6)) )**2
            #cost += O_weight * cs.fmax(0, (1.0 / (cs.fabs(d-obs_r) + 1e-6))**2 )

    # Terminal step application
    #e_T = X_n[[0, 2]] - x_ref[(pred_h - 1) * 2 : pred_h * 2] # Extract x and y positions from X_n
    #cost += cs.bilin(T_m, e_T, e_T)


    x_v_min, x_v_max = 0.0, 0.5
    y_v_min, y_v_max = -0.01, 0.01
    z_v_min, z_v_max = -0.5, 0.5

    u_min = [x_v_min, y_v_min, z_v_min] * cont_h
    u_max = [x_v_max, y_v_max, z_v_max] * cont_h
    bounds = og.constraints.Rectangle(u_min, u_max)

    # Build Definition
    problem = og.builder.Problem(u_sequence, params, cost).with_constraints(bounds)

    build_config = og.config.BuildConfiguration()\
        .with_build_directory("mpc_build")\
        .with_build_mode("release")\
        .with_tcp_interface_config()

    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-6) \
        .with_initial_tolerance(1e-6)\
        .with_max_outer_iterations(20) \
        .with_max_inner_iterations(50)

    meta = og.config.OptimizerMeta().with_optimizer_name("optimized_mpc")

    # Run code generator
    builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config)
    builder.build()