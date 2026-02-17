import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ModelPredictiveController import *
from first_order_delay_model import *
import numpy as np


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
        self.z = 0




    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0     # forward velocity (m/s)
        msg.linear.y = 0.0
        msg.angular.z = 0.0    # yaw rate (rad/s)
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        # Extract position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z


def main():
    rclpy.init()

    n_inputs = 2
    n_states = 2



    Q = 10 # state cost
    R = 3 # control cost
    R_d = 10 # smoothness cost
    T = 0.3 # terminal state cost
    O = 0 # obstacle cost
    pred_h = 10
    cont_h = 10

    tau = 0.01
    tau_u = 10
    T = 0.1
    k = 0.8
    model = None
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
        node.destroy_node()
        rclpy.shutdown()
main()