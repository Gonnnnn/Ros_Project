# /turtle1/pose 60hz x y theta lin_vel ang_vel
# /turtle2/pose
# /turtle2/cmd_vel 30hz max

import dwa_pathplanning_turtlesim.dynamic_window_approach as dwa

import sys
# import argparse

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

class DWA_Planner(Node):

    def __init__(self):
        super().__init__('DWA_Planner')

        # for Twist
        self.twist = Twist()
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()

        # for Pose
        self.pose = Pose()

        # Variables from dynammic_window_approach
        # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
        # goal position [x(m), y(m)]
        self.goal = np.array([[8.58, 2.5],
                          [2.5, 8.58],
                          [2.5, 2.5],
                          [8.58, 8.58]
                          ])

        self.config = dwa.Config()
        self.config.robot_type = dwa.RobotType.circle
        # *************ob has to be changed later*******************************************************
        self.ob = np.array([[0, 0]])
        # self.ob = self.config.ob

        # input [forward speed(m/s), yaw_rate(rad/s)]
        self.u = []
        self.predicted_trajectory = []
        self.dist_to_goal = 0

        self.idx_of_goal = 0

###################################

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

###################################

        self.DWA_Planner_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.subscribe,
            qos_profile)
        
        self.DWA_Planner_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            qos_profile)


        self.timer = self.create_timer(1, self.publish)

        self.count = 0


    def subscribe(self, msg):
        self.pose.x = msg.x
        self.pose.y = msg.y
        self.pose.theta = msg.theta
        self.pose.linear_velocity = msg.linear_velocity
        self.pose.angular_velocity = msg.angular_velocity


    def publish(self):


###################################

        self.u, self.predicted_trajectory = dwa.dwa_control(self.x, self.config, self.goal[self.idx_of_goal], self.ob)

        self.x[0], self.x[1], self.x[2], self.x[3], self.x[4] = self.pose.x, self.pose.y, self.pose.theta, self.pose.linear_velocity, self.pose.angular_velocity

        # check reaching goal
        self.dist_to_goal = math.hypot(self.x[0] - self.goal[self.idx_of_goal][0], self.x[1] - self.goal[self.idx_of_goal][1])
        if self.dist_to_goal <= self.config.robot_radius:
            if self.idx_of_goal == len(self.goal) - 1:
                self.idx_of_goal = 0
            else:
                self.idx_of_goal += 1


###################################

        self.lin_vec.x, self.lin_vec.y, self.lin_vec.z = self.u[0] * math.cos(self.x[2]), self.u[0] * math.sin(self.x[2]), 0.0
        self.ang_vec.x, self.ang_vec.y, self.ang_vec.z = 0.0, 0.0, self.u[1]

        self.twist.linear, self.twist.angular = self.lin_vec, self.ang_vec
        self.DWA_Planner_pub.publish(self.twist)
        
        self.count += 1


###################################
        
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