#!/usr/bin/env python3
"""
Launch file for visualizing RPLidar data in RViz
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package share directory
    pkg_dir = get_package_share_directory('testbot_sensor')
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'lidar_view.rviz')

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        rviz_node
    ])