import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kseikoff/ros2_ws/src/ekf_slam_robot/install/ekf_slam_robot'
