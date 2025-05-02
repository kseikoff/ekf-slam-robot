#!/bin/zsh
colcon build --symlink-install
source /opt/ros/humble/setup.zsh
source install/setup.sh
source /usr/share/gazebo/setup.sh
ros2 pkg list | grep ekf_slam_robot
ros2 pkg executables ekf_slam_robot
