#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription_cmd = self.create_subscription(
            Twist,
            '/desired_cmd_vel',
            self.cmd_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_threshold = 0.3
        self.front_angles = range(80, 100)
        self.current_cmd = Twist()


    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        front_distances = ranges[self.front_angles]
        min_distance = np.min(front_distances)

        twist = Twist()
        if min_distance < self.obstacle_threshold:
            left_avg = np.mean(ranges[0:80])
            right_avg = np.mean(ranges[100:180])

            if left_avg > right_avg:
                twist.angular.z = 0.5
            else:
                twist.angular.z = -0.5
        else:
            twist.linear.x = self.current_cmd.linear.x
            twist.angular.z = self.current_cmd.angular.z

        self.publisher_.publish(twist)


    def cmd_callback(self, msg):
        self.current_cmd = msg


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f'Exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()