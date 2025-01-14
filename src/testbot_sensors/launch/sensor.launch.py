#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # RPLidar node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'scan_frequency': 10.0
            }],
            output='screen'
        ),

        # Lidar processor node
        Node(
            package='testbot_sensors',
            executable='lidar_processor',
            name='lidar_processor',
            parameters=[{
                'scan_topic': 'scan',
                'min_obstacle_distance': 0.2,
                'safety_zone_radius': 0.3
            }],
            output='screen'
        ),

        # Lidar visualizer node
        Node(
            package='testbot_sensors',
            executable='lidar_visualizer',
            name='lidar_visualizer',
            parameters=[{
                'max_points': 100,
                'point_scale': 0.05
            }],
            output='screen'
        ),

        # Lidar monitor node
        Node(
            package='testbot_sensors',
            executable='lidar_monitor',
            name='lidar_monitor',
            output='screen'
        )
    ])