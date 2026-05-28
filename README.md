
# MPC controller for Unitree G2 Humanoid

# Usage

1. Start vrpn_mocap: /home/unitree/launch_vrpn_mocap.sh
2. Build MPC: python3 build_optimized_mpc.py
3. Start MPC: python3 optimized_controller.py
4. Launch the velocity node

# Code

## build_optimized_mpc.py

This script compiles the MPC using OpenGen.

Contains all the MPC parameters and the cost function.

Rust has to be installed along with the required python packages.

The **params** variable defines the input to the solver. See the order and the size of x_init x_ref and obs_ref. They have to align later when the solver is called.

**Edit this script when:**
1. Tuning model parameters
2. Tuning controller parameters
3. Changing number of obstacles
4. Changing solver parameters
5. Changing controller input constraints

**Always re-run the script after changing a parameter**

**ENCOUNTERED ERROR when starting the script:**

ImportError: cannot import name 'files' from 'importlib.resources'

**FIX:**

1. pip install importlib_resources
2. Change importlib.resources to importlib_resources in this file: /home/unitree/.local/lib/python3.8/site-packages/opengen/definitions.py


**IMPORTANT NOTE: If moving obstacles are to be defined, the cost function has to be edited**

## optimized_controller.py

This script starts the compiled MPC and publishes Twist messages.

First builds the TCP connection with the solver in the variable **mng**.

HighLevelController contains the subscribers and publishers of ros messages.

**Class HighLevelController:**

publish_cmd():

First builds the input to the solver **x_nr**.

The solver expects a flat array which has to align with the **params** variable in build_optimized_mpc.py.

The input is given to the solver and the solver gives the entire sequence of inputs. The first input is published as a ros2 topic.

goal_pose_callback():

For updating the goal.

Takes a PoseArray with respect to the body frame and updates the goal to be in the world frame

obstacle_pose_callback(): **!!OBS!! CURRENTLY ONLY WORKS FOR ONE OBSTACLE**

Same as goal_pose_callback() but for obstacles



========================================

Define self.obstacles like this: **(ASSUMES STATIC OBSTACLES)**

Obstacle 1: x = 1.0 y = 1.0

Obstacle 2: x = 2.0 y = 2.0

self.obstacles = [1.0,1.0,2.0,2.0]


**If something in the solver breaks:**

Check for solver error with print(solver_status.get().message)

Check that dimensions of the variable x_nr in the function publish_cmd align with what the solver expects. Example: does the solver expect 2 obstacles, but only one is defined in self.obstacles?

Check that the subscribers and publishers are set to listen/publish the correct messages

