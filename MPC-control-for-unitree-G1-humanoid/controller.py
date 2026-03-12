import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from ModelPredictiveController import *
from first_order_delay_model import *
import numpy as np
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped




class HighLevelController(Node):
    def __init__(self,controller):
        super().__init__('high_level_controller')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/g2_cmd_vel',10)

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
        #self.odom_subscriber_ = self.create_subscription(Odometry, '/odom',self.odom_callback,10)
        #self.g1emil_twist_subscriber_ = self.create_subscription(TwistStamped, '/vrpn_mocap/g1Emil1/twist',self.g1emiltwist_callback,qos)
        #self.g1emil_pose_subscriber_ = self.create_subscription(PoseStamped, '/vrpn_mocap/g1Emil1/pose',self.g1emilpose_callback,qos)
        self.human_pose_subscriber_ = self.create_subscription(PoseArray, '/human_pose',self.humanpose_callback,qos_poses)
        self.object_pose_subscriber_ = self.create_subscription(PoseArray, '/object_pose',self.objectpose_callback,qos_poses)
        #self.g1_pose_subscriber_ = self.create_subscription(PoseStamped, '/vicon/g1/g1',self.g1pose_callback,10)

        self.controller = controller

        self.x = 0
        self.x_twist = 0
        self.y = 0
        self.y_twist = 0
        self.z = 0
        self.z_twist = 0

        self.robot_coords = []
        self.mpc_path = []

        self.u0 = np.zeros(controller.cont_h*controller.n_inputs)

        self.goal = [-2,0]

    def publish_cmd(self):
        msg = Twist()

        goal = self.goal

        dy = goal[1] - self.y
        dx = goal[0] - self.x

        dz = math.atan2(dy,dx)-self.z
        dz = dz *0.1


        if abs(dy) < 1 and abs(dx) < 1:
            #self.goal = [4,6]
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        else:

            x_r = [goal] * self.controller.pred_h

            x = np.array([
                self.x,
                self.x_twist,
                self.y,
                self.y_twist,
                self.z,
                self.z_twist
                ])

            u,mpc_path,u0 = self.controller.next_u(x,x_r,self.u0)

            #self.u0 = u0

            mpc_path = mpc_path.reshape(-1,3)
            mpc_path_x = mpc_path[:,0]
            mpc_path_y = mpc_path[:,1]
            mpc_path_z = mpc_path[:,2]

            self.mpc_path.append([mpc_path_x,mpc_path_y,mpc_path_z])
            self.robot_coords.append([self.x,self.y,self.z])

            print(f"U: {u}")

            x_cmd = u[0]
            y_cmd = u[1]
            z_cmd = u[2]

            print(f"Goal: x: {goal[0]:4.2f} y: {goal[1]:4.2f}")

            print(f"Pos: x: {self.x:4.2f} y: {self.y:4.2f} z: {self.z:4.2f}")

            print(f"Cmd_vel: x: {x_cmd:4.2f} y: {y_cmd:4.2f} z: {z_cmd:4.2f}")

            #msg.linear.x = x_cmd
            #msg.linear.y = y_cmd
            #msg.angular.z = z_cmd
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        self.cmd_vel_publisher_.publish(msg)


    def odom_callback(self, msg):
        # Extract position
        self.x = msg.pose.position.x
        #self.y = msg.pose.position.y
        #z = msg.pose.pose.orientation.z
        #x = msg.pose.pose.orientation.x
        #y = msg.pose.pose.orientation.y
        #w = msg.pose.pose.orientation.w


        #self.z = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))



        #self.x_twist = msg.twist.twist.linear.x
        #self.y_twist = msg.twist.twist.linear.y
        #self.z_twist = msg.twist.twist.angular.z

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

    def g1pose_callback(self,msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        z = msg.pose.orientation.z
        x = msg.pose.orientation.x
        y = msg.pose.orientation.y
        w = msg.pose.orientation.w

        self.z = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

    def humanpose_callback(self,msg):

        x_bf = msg.poses.position.x
        y_bf = msg.poses.position.y

        x_bf = np.cos(self.z)*x_bf - np.sin(self.z)*y_bf
        y_bf = np.sin(self.z)*x_bf + np.cos(self.z)*y_bf

        x_wf = x_bf+self.x
        y_wf = y_bf+self.y

        self.goal = [x_wf, y_wf]

        print(f"Goal: x: {self.goal[0]:>10.2f} y: {self.goal[1]:>10.2f}")
        print(f"Position: x: {self.x:>10.2f} y: {self.y:>10.2f}")
    def humanpose_callback(self,msg):

        x_bf = msg.poses.position.x
        y_bf = msg.poses.position.y

        x_bf = np.cos(self.z)*x_bf - np.sin(self.z)*y_bf
        y_bf = np.sin(self.z)*x_bf + np.cos(self.z)*y_bf

        x_wf = x_bf+self.x
        y_wf = y_bf+self.y

        print(f"Goal: x: {self.goal[0]:>10.2f} y: {self.goal[1]:>10.2f}")
        print(f"Position: x: {self.x:>10.2f} y: {self.y:>10.2f}")






def main():
    rclpy.init()

    model = first_order_delay_model
    mpc = ModelPredictiveController(model)
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

        mpc_paths = np.array(node.mpc_path)

        np.save("/code/mpc_paths.npy",mpc_paths)
        np.save("/code/rob_coords.npy",coords)
        node.destroy_node()
        rclpy.shutdown()
main()


