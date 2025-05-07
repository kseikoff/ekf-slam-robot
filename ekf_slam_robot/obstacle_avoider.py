import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('ekf_slam_avoider')

        self.create_subscription(LaserScan, '/lidar_controller/out', self.scan_callback, 10)
        self.create_subscription(Twist, '/desired_cmd_vel', self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_cmd = Twist()
        self.pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.cov = np.eye(3) * 0.1
        self.landmarks = []  # list of [x, y]

        self.dt = 0.1
        self.q = 0.05
        self.r = 0.2

        self.obstacle_threshold = 0.7
        self.front_angles = range(80, 100)

        self.get_logger().info("obstacle avoider initialized")

    def cmd_callback(self, msg: Twist):
        self.current_cmd = msg
        self.predict_step()

    def predict_step(self):
        x, y, theta = self.pose
        v = self.current_cmd.linear.x
        w = self.current_cmd.angular.z

        theta_new = theta + w * self.dt
        x_new = x + v * np.cos(theta) * self.dt
        y_new = y + v * np.sin(theta) * self.dt

        self.pose = np.array([x_new, y_new, theta_new])

        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0, 1]
        ])

        self.cov = F @ self.cov @ F.T + np.eye(3) * self.q

    def get_safe_direction(self, ranges, num_bins=18):
        sector_size = len(ranges) // num_bins
        safe_scores = []

        for i in range(num_bins):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size if i < num_bins - 1 else len(ranges)
            sector_ranges = ranges[start_idx:end_idx]
            avg_distance = np.mean(sector_ranges[np.isfinite(sector_ranges)])
            safe_scores.append(avg_distance)

        safe_scores = np.array(safe_scores)
        safe_direction = np.argmax(safe_scores)

        return safe_direction, safe_scores

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=3.5, posinf=3.5, neginf=3.5)
        angles = np.linspace(-np.pi/2, np.pi/2, len(ranges))

        new_landmarks = self.extract_landmarks(ranges, angles)
        self.update_step(new_landmarks)

        safe_dir, _ = self.get_safe_direction(ranges)
        twist = Twist()

        target_angle = (safe_dir - 9) * (np.pi / 9)
        angular_velocity = np.clip(target_angle * 1.0, -1.0, 1.0)

        front_clear = np.min(ranges[80:100]) > self.obstacle_threshold

        if front_clear:
            twist.linear.x = self.current_cmd.linear.x
            twist.angular.z = angular_velocity * 0.7
        else:
            twist.linear.x = max(0.0, self.current_cmd.linear.x * 0.3)
            twist.angular.z = angular_velocity

        self.cmd_pub.publish(twist)

    def extract_landmarks(self, ranges, angles):
        grads = np.gradient(ranges)
        peaks = np.where(np.abs(grads) > 0.4)[0]
        landmarks = []

        x_r, y_r, theta = self.pose

        for i in peaks:
            r = ranges[i]
            if np.isinf(r) or r < 0.2 or r > 5.0:
                continue
            angle = angles[i]
            lx = x_r + r * np.cos(theta + angle)
            ly = y_r + r * np.sin(theta + angle)
            landmarks.append((lx, ly))
        return landmarks

    def update_step(self, observations):
        for obs in observations:
            ox, oy = obs
            matched = False
            for i, (lx, ly) in enumerate(self.landmarks):
                dist = np.hypot(ox - lx, oy - ly)
                if dist < 0.5:
                    new_x = (lx + ox) / 2
                    new_y = (ly + oy) / 2
                    self.landmarks[i] = (new_x, new_y)
                    matched = True
                    break
            if not matched:
                self.landmarks.append((ox, oy))
        self.get_logger().info(f"Landmarks: {len(self.landmarks)}")

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
