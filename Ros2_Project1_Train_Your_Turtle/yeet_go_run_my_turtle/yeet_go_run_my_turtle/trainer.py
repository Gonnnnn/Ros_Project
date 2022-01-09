import sys
import argparse

from msg_pkg.msg import RVD
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy



class Trainer(Node):

    def __init__(self, radius, lin_vel, cw):
        super().__init__('trainer')

        self.radius = radius
        self.lin_vel = lin_vel
        self.cw = cw

        self.lin_x = 0.0
        self.lin_y = 0.0
        self.lin_z = 0.0
        self.ang_x = 0.0
        self.ang_y = 0.0
        self.ang_z = 0.0

        self.lin_vec = None
        self.ang_vec = None
        self.twist = None

        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.trainer_sub = self.create_subscription(
            RVD,
            'circular_motion',
            self.subscribe,
            qos_profile)
        
        self.trainer_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            qos_profile)

        self.timer = self.create_timer(1, self.publish)

        self.count = 0
        

    def subscribe(self, msg):
        self.radius = msg.radius
        self.lin_vel = msg.linear_velocity
        self.cw = msg.clockwise

    def publish(self):
        self.twist = Twist()
        self.lin_vec = Vector3()
        self.ang_vec = Vector3()

        if(self.count == 0 or self.lin_x != self.lin_vel):
            self.get_logger().info('#################################################\n')
            if(self.lin_vel < 2.78):
                self.get_logger().info(f'Come on.. {self.lin_vel}? It\'s way too slow\n')
            elif(self.lin_vel >= 2.78 and self.lin_vel < 21.4):
                self.get_logger().info('Welp not bad but he\'s still slower than Usain Bolt\n')
            elif(self.lin_vel > 21.4):
                self.get_logger().info('USAIN BOLT! USAIN BOLT! USAIN BOLT!')
                self.get_logger().info('DO YOU KNOW WHY HE\'S THE FASTEST IN THE WORLD?')
                self.get_logger().info('Cuz he made it in the end\n')
            self.get_logger().info('#################################################\n')


        self.lin_x = self.lin_vel

        if(self.radius == 0.0):
            self.ang_z = 0
        elif(self.radius != 0.0):
            omega = self.lin_x / self.radius
            self.ang_z = omega if self.cw == True else -omega

        self.lin_vec.x, self.lin_vec.y, self.lin_vec.z = self.lin_x, self.lin_y, self.lin_z
        self.ang_vec.x, self.ang_vec.y, self.ang_vec.z = self.ang_x, self.ang_y, self.ang_z

        self.twist.linear, self.twist.angular = self.lin_vec, self.ang_vec
        self.trainer_pub.publish(self.twist)
        
        kcal_per_sec = self.lin_vel / 2.78 * 480 / 60 / 60
        kcal = self.count * kcal_per_sec
        self.get_logger().info(f'Radius : {self.radius}, linear velocity : {self.lin_vel}, clockwise : {self.cw}, count : {self.count}')
        self.get_logger().info(f'Your cutie cutie fat turtle has burnt {kcal} kcal!\n')
        
        self.count += 1

def argment_parser():
    parser = argparse.ArgumentParser(description='Insert --r=radius(m, float), --v=linear velocity(m/s, float), --cw=direction(t if cw, f if ccw.')
    parser.add_argument('-r', help='Radius // Unit: m // Ex) 3.2m -> --r=3.2', type=float, dest='radius', default=1.0)
    parser.add_argument('-v', help='Linear Velocity // Unit: m/s // Ex) 1.7m/s -> --v=1.7', type=float, dest='velocity', default=0.0)
    parser.add_argument('-cw', help='Direction // Ex) clockwise -> --cw=t', type=str, dest='direction', default='t')
    
    args = parser.parse_args()

    if(args.direction == 't'):
        cw = True
    elif(args.direction == 'f'):
        cw = False

    return args.radius, args.velocity, cw

def main(args=None):
    rclpy.init(args=args)
    radius, lin_vel, cw = argment_parser()

    try:
        trainer = Trainer(radius, lin_vel, cw)
        try:
            rclpy.spin(trainer)
        except KeyboardInterrupt:
            trainer.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            trainer.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()