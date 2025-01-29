#!/usr/bin/env python3
"""
Launch file for visualizing SLAM in RViz
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_testbot_slam = get_package_share_directory('testbot_slam')
    
    # Configure RViz
    rviz_config = os.path.join(pkg_testbot_slam, 'rviz', 'slam.rviz')
    
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