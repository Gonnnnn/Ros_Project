# /turtle1/pose
# /turtle2/pose
# /turtle2/cmd_vel

import dwa_pathplanning_turtlesim.dynamic_window_approach as dwa

import sys
# import argparse

import math
from enum import Enum
import numpy as np

from msg_pkg.msg import RVD
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

class DWA_Planner(Node):

    def __init__(self):
        super().__init__('DWA_Planner')

        # self.radius = radius
        # self.lin_vel = lin_vel
        # self.cw = cw

        # self.lin_x = 0.0
        # self.lin_y = 0.0
        # self.lin_z = 0.0
        # self.ang_x = 0.0
        # self.ang_y = 0.0
        # self.ang_z = 0.0

        self.lin_vec = None
        self.ang_vec = None
        self.twist = None

###################################

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
        self.ob = self.config.ob

        # input [forward speed, yaw_rate]
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

        # self.DWA_Planner_sub = self.create_subscription(
        #     RVD,
        #     'circular_motion',
        #     self.subscribe,
        #     qos_profile)
        
        self.DWA_Planner_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            qos_profile)


        self.timer = self.create_timer(1, self.publish)

        self.count = 0

###################################

    # def subscribe(self, msg):
    #     self.radius = msg.radius
    #     self.lin_vel = msg.linear_velocity
    #     self.cw = msg.clockwise

    def publish(self):
        self.twist = Twist()
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()

###################################

        self.u, self.predicted_trajectory = dwa.dwa_control(self.x, self.config, self.goal[self.idx_of_goal], self.ob)
        self.x = dwa.motion(self.x, self.u, self.config.dt)  # simulate robot

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