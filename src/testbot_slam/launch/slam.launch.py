#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_testbot_slam = get_package_share_directory('testbot_slam')
    
    # Declare launch arguments and get parameter file path
    slam_params_file = os.path.join(pkg_testbot_slam, 'config', 'slam_toolbox_params.yaml')
    
    if not os.path.exists(slam_params_file):
        raise FileNotFoundError(f'Cannot find parameters file: {slam_params_file}')

    # Start SLAM Toolbox node
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    # Create and return launch description
    return LaunchDescription([
        start_slam_toolbox_node
    ])