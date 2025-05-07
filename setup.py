import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ekf_slam_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.gazebo')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kseikoff',
    maintainer_email='kseikoff@todo.todo',
    description='EKF SLAM Robot Impl',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_publisher = ekf_slam_robot.cmd_vel_publisher:main',
            'obstacle_avoider = ekf_slam_robot.obstacle_avoider:main'
        ],
    },
)
