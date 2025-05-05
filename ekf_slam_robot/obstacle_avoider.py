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
            '/lidar_controller/out', # should be /diff_drive_robot/laser_1/scan but it does not work (no idea why)
            self.scan_callback,
            10)
        self.subscription_cmd = self.create_subscription(
            Twist,
            '/desired_cmd_vel',
            self.cmd_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_threshold = 0.7
        self.front_angles = range(80, 100)
        self.current_cmd = Twist()
        self.avoiding = False


    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        front_distances = ranges[self.front_angles]
        min_distance = np.min(front_distances)

        twist = Twist()
        if min_distance < self.obstacle_threshold:
            self.avoiding = True

            left_avg = np.mean(ranges[0:80])
            right_avg = np.mean(ranges[100:180])

            if left_avg > right_avg:
                twist.angular.z = 0.5
                self.get_logger().info(f'Turning left: z={twist.angular.z}')
            else:
                twist.angular.z = -0.5
                self.get_logger().info(f'Turning right: z={twist.angular.z}')
        else:
            self.avoiding = False

            twist.linear.x = self.current_cmd.linear.x
            twist.angular.z = self.current_cmd.angular.z
            self.get_logger().info(f'Going forward: x={twist.linear.x}, z={twist.linear.z}')

        self.publisher_.publish(twist)


    def cmd_callback(self, msg: Twist):
        self.get_logger().info('Connection with desired_cmd_vel node established', once=True)
        if not self.avoiding:
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