
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray
#from vicon_receiver.msg import Position
from nav_msgs.msg import Odometry, Path
from first_order_delay_model import *
import numpy as np
import math
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
import opengen as og

from build_optimized_mpc import pred_h, n_inputs, n_obstacles





class HighLevelController(Node):
    def __init__(self,mng):
        super().__init__('high_level_controller')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/g2_cmd_vel',10)
        #self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel',10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        #qos profile for harrys pose array
        qos_poses = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,  # Use KEEP_LAST as default for unknown
            depth=10,                          # Arbitrary depth since history is UNKNOWN
            durability=DurabilityPolicy.VOLATILE
        )

        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.timer2 = self.create_timer(0.1, self.publish_mpc_path)

        #self.odom_subscriber_ = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        self.g1emil_twist_subscriber_ = self.create_subscription(TwistStamped, '/vrpn_mocap/g1/twist',self.g1emiltwist_callback,qos)
        self.g1emil_pose_subscriber_ = self.create_subscription(PoseStamped, '/vrpn_mocap/g1/pose',self.g1emilpose_callback,qos)
        #self.champ_pose_subscriber_ = self.create_subscription(Odometry, '/odom',self.champ_pose_callback,qos)
        #self.human_pose_subscriber_ = self.create_subscription(PoseArray, '/human',self.humanpose_callback,qos_poses)
        #self.object_pose_subscriber_ = self.create_subscription(PoseArray, '/obstacle',self.objectpose_callback,qos_poses)
        #self.g1_pose_subscriber_ = self.create_subscription(Position, '/vicon/g2/g2',self.g1pose_callback,qos_poses)
        #self.box_subscriber_ = self.create_subscription(PoseStamped, '/vrpn_mocap/box_mic/pose',self.box_callback,qos_poses)

        self.mng = mng

        self.x = 0
        self.x_twist = 0
        self.y = 0
        self.y_twist = 0
        self.z = 0
        self.z_twist = 0

        self.robot_coords = []
        self.mpc_path = []

        self.goal = [0.0, 0.0]

        if n_obstacles > 0:
            self.obstacles = [0.0,1.0]

        self.received_goal = False

        self.received_obstacle = False



    def publish_cmd(self):
        msg = Twist()


        goal = self.goal

        dy = goal[1] - self.y
        dx = goal[0] - self.x

        dz = math.atan2(dy,dx)-self.z


        if np.sqrt(dx**2 + dy**2) < 0.1:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            print("Goal reached. Stopping robot.")
        else:
            x_r = goal * pred_h

            obs = self.obstacles * pred_h

            x = [
                self.x,
                self.x_twist,
                self.y,
                self.y_twist,
                self.z,
                self.z_twist
                ]

            x_nr = x + x_r + obs
            solver_status = self.mng.call(x_nr, None)

            us = solver_status['solution']

            u = us[0:3]
            x_cmd = u[0]
            y_cmd = u[1]
            z_cmd = u[2]

            print(f"Goal: x: {goal[0]:4.2f} y: {goal[1]:4.2f}")

            print(f"Pos: x: {self.x:4.2f} y: {self.y:4.2f} z: {self.z:4.2f}")

            print(f"Cmd_vel: x: {x_cmd:4.2f} y: {y_cmd:4.2f} z: {z_cmd:4.2f}")

            print(f"Obstacle: x: {obs[0]:4.2f} y: {obs[1]:4.2f}")
            #print(f"Box: x: {self.box[0]:4.2f} y: {self.box[1]:4.2f}")

            msg.linear.x = x_cmd
            msg.linear.y = y_cmd
            msg.angular.z = z_cmd
            #msg.linear.x = 0.0
            #msg.linear.y = 0.0
            #msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)

    def g1emilpose_callback(self, msg):


        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        z = msg.pose.orientation.z
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        w = msg.pose.orientation.w


        self.z = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def g1emiltwist_callback(self,msg):

        self.x_twist = msg.twist.linear.x
        self.y_twist = msg.twist.linear.y
        self.z_twist = msg.twist.angular.z

    def humanpose_callback(self,msg):

        if not self.received_goal:


            x_bf = msg.poses[0].position.x
            y_bf = msg.poses[0].position.y

            x_bf = np.cos(self.z)*x_bf - np.sin(self.z)*y_bf
            y_bf = np.sin(self.z)*x_bf + np.cos(self.z)*y_bf

            x_wf = x_bf+self.x
            y_wf = y_bf+self.y

            self.goal = [x_wf, y_wf]
            self.received_goal = True
        else:
            return

    def objectpose_callback(self,msg):

        if not self.received_obstacle:

            x_bf = msg.poses[0].position.x
            y_bf = msg.poses[0].position.y

            x_bf = np.cos(self.z)*x_bf - np.sin(self.z)*y_bf
            y_bf = np.sin(self.z)*x_bf + np.cos(self.z)*y_bf

            x_wf = x_bf+self.x
            y_wf = y_bf+self.y

            self.obstacles = [x_wf,y_wf]
            self.received_obstacle = True
        else:
            return

    def box_callback(self,msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.box = [x,y]





def main():
    rclpy.init()

    mng = og.tcp.OptimizerTcpManager("mpc_build/optimized_mpc")

    mng.start()

    node = HighLevelController(mng)
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

        mng.kill()
        node.destroy_node()
        rclpy.shutdown()
main()


