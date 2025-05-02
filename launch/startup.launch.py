from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import xacro

import os


def generate_launch_description():
    pkg_path = get_package_share_directory('ekf_slam_robot')
    world_path = os.path.join(pkg_path, 'worlds', 'empty.world')
    urdf_path = os.path.join(pkg_path, 'urdf/robot.xacro')

    robot_description = xacro.process_file(urdf_path).toxml()
    xml = robot_description.replace('"', '\\"')
    spawn_args = '{name: \"ekf_slam_robot\", xml: \"' + xml + '\" }'

    gazeboworld = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path,
              '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gz_plugin_path = AppendEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    spawnmodel = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_entity',
             'gazebo_msgs/SpawnEntity', spawn_args],
        output='screen'
    )

    jspg_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                    'use_sim_time': True}]
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--ros-args', '-p', 'use_sim_time:=true']
    )
    
    ekf_cfg_path = os.path.join(
        pkg_path,
        'config',
        'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_cfg_path]
    )
    
    obstacle_avoider_node = Node(
        package='ekf_slam_robot',
        executable='obstacle_avoider',
        name='obstacle_avoider_node',
        output='screen'
    )
    cmd_vel_publisher_node = Node(
        package='ekf_slam_robot',
        executable='cmd_vel_publisher',
        name='cmd_vel_publisher_node',
        output='screen'
    )
    
    ld = LaunchDescription()

    ld.add_action(jspg_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz2_node)

    ld.add_action(gz_plugin_path)
    ld.add_action(gazeboworld)
    ld.add_action(spawnmodel)

    ld.add_action(ekf_node)
    ld.add_action(obstacle_avoider_node)
    ld.add_action(cmd_vel_publisher_node)

    return ld