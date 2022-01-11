import dwa_pathplanning_turtlesim.dynamic_window_approach as dwa

import sys

import math
from enum import Enum
import numpy as np

from msg_pkg.msg import RVD
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class turtle():
    def __init__(self, number):
        ## msg - Twist
        self.twist = Twist()
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()

        ## msg - Pose
        self.pose = Pose()

        ## Variables from dynammic_window_approach
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        # goal position [x(m), y(m)]
        self.goal = np.array([[8.58, 2.5],
                          [2.5, 8.58],
                          [2.5, 2.5],
                          [8.58, 8.58]
                          ])
        self.config = dwa.Config()
        self.config.robot_type = dwa.RobotType.circle

        # input [forward speed(m/s), yaw_rate(rad/s)]
        self.u = []
        self.dist_to_goal = 0
        self.idx_of_goal = 0

        ## etc
        self.notify = True
        self.number = number

    def calc(self):
        # update the current location
        self.x[0], self.x[1], self.x[2], self.x[3], self.x[4] = self.pose.x, self.pose.y, self.pose.theta, self.pose.linear_velocity, self.pose.angular_velocity

        # dwa
        self.u, _ = dwa.dwa_control(self.x, self.config, self.goal[self.idx_of_goal], self.config.ob)
        if(self.notify == True):
            print("The turtle{0} is currently going to the goal {1}".format(self.number, self.idx_of_goal + 1))
            self.notify = False

        # check reaching goal
        self.dist_to_goal = math.hypot(self.x[0] - self.goal[self.idx_of_goal][0], self.x[1] - self.goal[self.idx_of_goal][1])
        if self.dist_to_goal <= self.config.robot_radius:
            print("The turtle{0} just reached the goal {1}".format(self.number, self.idx_of_goal + 1))
            self.idx_of_goal += 1
            self.idx_of_goal %= len(self.goal)
            self.notify = True

        # publish Twist
        self.lin_vec.x, self.lin_vec.y, self.lin_vec.z = self.u[0], 0.0, 0.0
        self.ang_vec.x, self.ang_vec.y, self.ang_vec.z = 0.0, 0.0, self.u[1]

        self.twist.linear, self.twist.angular = self.lin_vec, self.ang_vec

        return self.twist
        

class DWA_Planner(Node):

    def __init__(self):
        super().__init__('DWA_Planner')

        self.turtle1 = turtle(1)
        self.turtle2 = turtle(2)

        ## qos_profile
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        ## subscriber and publisher
        self.DWA_Planner_sub1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscribe1,
            qos_profile)
    
        # self.DWA_Planner_pub1 = self.create_publisher(
        #     Twist,
        #     '/turtle1/cmd_vel',
        #     qos_profile)

        # self.timer1 = self.create_timer(0.1, self.publish1)

        self.DWA_Planner_sub2 = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.subscribe2,
            qos_profile
        )

        self.DWA_Planner_pub2 = self.create_publisher(
            Twist,
            '/turtle2/cmd_vel',
            qos_profile)
        
        self.timer2 = self.create_timer(0.1, self.publish2)



    def subscribe1(self, msg):
        self.turtle1.pose.x = msg.x
        self.turtle1.pose.y = msg.y
        self.turtle1.pose.theta = msg.theta
        self.turtle1.pose.linear_velocity = msg.linear_velocity
        self.turtle1.pose.angular_velocity = msg.angular_velocity

        self.turtle2.config.ob[0][0] = msg.x
        self.turtle2.config.ob[0][1] = msg.y
    

    # def publish1(self):
    #     # update the current location
    #     result = self.turtle1.calc()
    #     self.DWA_Planner_pub1.publish(result)



    def subscribe2(self, msg):
        self.turtle2.pose.x = msg.x
        self.turtle2.pose.y = msg.y
        self.turtle2.pose.theta = msg.theta
        self.turtle2.pose.linear_velocity = msg.linear_velocity
        self.turtle2.pose.angular_velocity = msg.angular_velocity

        self.turtle1.config.ob[0][0] = msg.x
        self.turtle1.config.ob[0][1] = msg.y


    def publish2(self):
        # update the current location
        result = self.turtle2.calc()
        self.DWA_Planner_pub2.publish(result)


def main(args=None):
    rclpy.init(args=args)

    try:
        dwa_planner = DWA_Planner()
        try:
            rclpy.spin(dwa_planner)
        except KeyboardInterrupt:
            dwa_planner.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            dwa_planner.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()