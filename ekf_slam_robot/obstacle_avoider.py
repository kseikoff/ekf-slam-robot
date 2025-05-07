#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.create_subscription(LaserScan, '/lidar_controller/out', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Twist, '/desired_cmd_vel', self.cmd_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obstacle_threshold = 0.7
        self.front_angles = range(80, 100)
        self.current_cmd = Twist()
        self.current_pose = np.zeros(3)

        self.get_logger().info("obstacle avoider node initialized")


    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        # angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # x_robot, y_robot, theta_robot = self.current_pose

        # for i in range(len(ranges)):
        #     r = ranges[i]
        #     phi = angles[i]

        #     if r < msg.range_min or r > msg.range_max:
        #         continue

        #     x_global = x_robot + r * np.cos(theta_robot + phi)
        #     y_global = y_robot + r * np.sin(theta_robot + phi)

        #     self.get_logger().info(f'Found object ({x_global:.2f}, {y_global:.2f})')

        front_distances = ranges[self.front_angles]
        min_distance = np.min(front_distances)

        twist = Twist()

        if min_distance < self.obstacle_threshold:
            left_min = np.min(ranges[0:80])
            right_min = np.min(ranges[100:180])

            if abs(left_min-right_min) < 0.2:
                twist.linear.x = -self.current_cmd.linear.x
                twist.angular.z = 0.5
                self.get_logger().info(f'Moving back: x={twist.linear.x}, z={twist.angular.z}')
            elif left_min > right_min:
                twist.angular.z = 0.5
                self.get_logger().info(f'Turning right: z={twist.angular.z},\
                                       left_min={left_min},\
                                       right_min={right_min}')
            else:
                twist.angular.z = -0.5
                self.get_logger().info(f'Turning left: z={twist.angular.z},\
                                       left_min={left_min},\
                                       right_min={right_min}')
        else:
            twist.linear.x = self.current_cmd.linear.x
            twist.angular.z = self.current_cmd.angular.z
            self.get_logger().info(f'Moving forward: x={twist.linear.x}, z={twist.angular.z}')

        self.cmd_pub.publish(twist)


    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        theta = self.quaternion_to_yaw(quat)
        self.current_pose = np.array([x, y, theta])


    def cmd_callback(self, msg: Twist):
        self.get_logger().info('Connection with node established', once=True)
        self.current_cmd = msg


    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()