#!/bin/zsh
source install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 launch ekf_slam_robot startup.launch.py
