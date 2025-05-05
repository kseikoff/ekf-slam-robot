#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/desired_cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.linear.x}, z={msg.linear.z}')


def main():
    rclpy.init()
    node = CmdVelPublisher()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()