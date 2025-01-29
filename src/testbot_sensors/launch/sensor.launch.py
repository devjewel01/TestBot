#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('testbot_sensors')
    
    # Load config files
    rplidar_config = os.path.join(pkg_dir, 'config', 'rplidar_params.yaml')
    filter_config = os.path.join(pkg_dir, 'config', 'laser_filter_params.yaml')

    # RPLidar node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[rplidar_config],
        remappings=[('scan', 'scan_raw')],
        output='screen'
    )

    # Laser filter node
    filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[filter_config],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan')
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        rplidar_node,
        filter_node
    ])