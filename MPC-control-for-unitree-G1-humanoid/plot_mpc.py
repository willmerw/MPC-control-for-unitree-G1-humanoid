import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# User-defined parameters
# -----------------------------
setpoint1 = np.array([2.0, 3.0])   # First goal [x, y]
setpoint2 = np.array([4.0, 6.0])   # Second goal [x, y]
rmse_threshold = 0.05              # Switching threshold

output_filename = "tracking_performance.png"

# -----------------------------
# Load data
# -----------------------------
robot_state = np.load("rob_coords.npy")   # shape (T,3) -> x,y,yaw
x = robot_state[:, 0]
y = robot_state[:, 1]
T = len(x)
time = np.arange(T)

# -----------------------------
# Position errors
# -----------------------------
error1_x = x - setpoint1[0]
error1_y = y - setpoint1[1]
error2_x = x - setpoint2[0]
error2_y = y - setpoint2[1]

# -----------------------------
# RMSE tracking logic with sliding window
# -----------------------------
window = 10  # number of steps for local RMSE
rmse_x = np.zeros(T)
rmse_y = np.zeros(T)
tracking_second = False

for k in range(T):
    if not tracking_second:
        start_idx = max(0, k-window+1)
        rmse_x[k] = np.sqrt(np.mean(error1_x[start_idx:k+1]**2))
        rmse_y[k] = np.sqrt(np.mean(error1_y[start_idx:k+1]**2))

        if np.sqrt(error1_x[k]**2 + error1_y[k]**2) < rmse_threshold:
            tracking_second = True
    else:
        start_idx = max(0, k-window+1)
        rmse_x[k] = np.sqrt(np.mean(error2_x[start_idx:k+1]**2))
        rmse_y[k] = np.sqrt(np.mean(error2_y[start_idx:k+1]**2))

# -----------------------------
# Plotting
# -----------------------------
fig, axs = plt.subplots(2, 2, figsize=(10, 8))

# 1. X position
axs[0, 0].plot(time, x, label="x position")
axs[0, 0].axhline(setpoint1[0], linestyle='--', label="Setpoint 1")
axs[0, 0].axhline(setpoint2[0], linestyle=':', label="Setpoint 2")
axs[0, 0].set_title("X Position vs Time")
axs[0, 0].set_xlabel("Time step")
axs[0, 0].set_ylabel("X position")
axs[0, 0].legend()
axs[0, 0].grid(True)

# 2. Y position
axs[0, 1].plot(time, y, label="y position")
axs[0, 1].axhline(setpoint1[1], linestyle='--', label="Setpoint 1")
axs[0, 1].axhline(setpoint2[1], linestyle=':', label="Setpoint 2")
axs[0, 1].set_title("Y Position vs Time")
axs[0, 1].set_xlabel("Time step")
axs[0, 1].set_ylabel("Y position")
axs[0, 1].legend()
axs[0, 1].grid(True)

# 3. RMSE X
axs[1, 0].plot(time, rmse_x)
axs[1, 0].set_title("RMSE - X")
axs[1, 0].set_xlabel("Time step")
axs[1, 0].set_ylabel("RMSE")
axs[1, 0].grid(True)

# 4. RMSE Y
axs[1, 1].plot(time, rmse_y)
axs[1, 1].set_title("RMSE - Y")
axs[1, 1].set_xlabel("Time step")
axs[1, 1].set_ylabel("RMSE")
axs[1, 1].grid(True)

plt.tight_layout()
plt.savefig(output_filename, dpi=300)
plt.close()

print(f"Saved figure as {output_filename}")