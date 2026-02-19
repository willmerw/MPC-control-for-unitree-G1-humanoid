import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ModelPredictiveController import *
from first_order_delay_model import *
import matplotlib.pyplot as plt
import numpy as np
import math


class HighLevelController(Node):
    def __init__(self,controller):
        super().__init__('high_level_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel',10)
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.subscriber_ = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        self.controller = controller

        self.x = 0
        self.x_twist = 0
        self.y = 0
        self.y_twist = 0
        self.z = 0
        self.z_twist = 0

        self.robot_coords = []
        self.mpc_path = []




    def publish_cmd(self):
        msg = Twist()


        goal = [4,2]



        yaw = self.z
        R = np.array([
        [np.cos(yaw),  np.sin(yaw)],
        [-np.sin(yaw), np.cos(yaw)]
        ])
        goal_bf = R @ goal

        dy = goal[1]-self.y
        dx = goal[0] - self.x
        z_goal = math.atan2(dy,dx)
        goal = (goal_bf[0],goal_bf[1],z_goal)
        x_r = [goal] * self.controller.pred_h
        input_bounds = [-0.2,0.2]

        x_bf, y_bf = R@np.array([self.x,self.y])
        xt_bf, yt_bf = R@np.array([self.x_twist,self.y_twist])
        """
        x = np.array([
            self.x,
            self.x_twist,
            self.y,
            self.y_twist,
            self.z,
            self.z_twist
            ])
        """
        x = np.array([
            x_bf,
            xt_bf,
            y_bf,
            yt_bf,
            self.z,
            self.z_twist
            ])
        u,mpc_path = self.controller.next_u(x,x_r,None,input_bounds)

        mpc_path = mpc_path.reshape(-1,3)
        mpc_path_x = mpc_path[:,0]
        mpc_path_y = mpc_path[:,1]
        mpc_path_z = mpc_path[:,2]

        self.mpc_path.append([mpc_path_x,mpc_path_y,mpc_path_z])
        self.robot_coords.append([self.x,self.y,self.z])

        x_cmd = u[0]
        y_cmd = u[1]
        z_cmd = u[2]

        msg.linear.x = x_cmd
        msg.linear.y = y_cmd
        msg.angular.z = z_cmd
        self.publisher_.publish(msg)


    def odom_callback(self, msg):
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        w = msg.pose.pose.orientation.w


        self.z = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        self.x_twist = msg.twist.twist.linear.x
        self.y_twist = msg.twist.twist.linear.y
        self.z_twist = msg.twist.twist.angular.z



def main():
    rclpy.init()

    n_inputs = 3
    n_states = 3



    Q = 10 # state cost
    R = 3 # control cost
    R_d = 10 # smoothness cost
    T = 0.3 # terminal state cost
    O = 0 # obstacle cost
    pred_h = 5
    cont_h = 10

    Ts = 0.1

    model = first_order_delay_model
    mpc = ModelPredictiveController(model,n_states,n_inputs,pred_h,cont_h,Q,R,R_d,T,O,0)
    node = HighLevelController(mpc)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Publish zero velocity before shutdown
        stop_msg = Twist()
        stop_msg.linear.x = 0.0     # forward velocity (m/s)
        stop_msg.linear.y = 0.0
        stop_msg.angular.z = 0.0
        node.publisher_.publish(stop_msg)
        node.get_logger().info("Stopping robot.")
    finally:
        coords = np.array(node.robot_coords)
        plt.axis([-4,4,-4,4])
        plt.plot(coords[:,0],coords[:,1])

        mpc_paths = np.array(node.mpc_path)

        print(mpc_paths)
        np.save("/code/mpc_paths.npy",mpc_paths)
        np.save("/code/rob_coords.npy",coords)
        node.destroy_node()
        rclpy.shutdown()
main()


