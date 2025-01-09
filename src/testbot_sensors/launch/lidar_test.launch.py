#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # RPLidar node 
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'scan_frequency': 10.0
            }],
            output='screen'
        ),

        # Basic visualization in RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 'src/testbot_sensors/config/lidar_test.rviz'],
            output='screen'
        ),

        # Your lidar processor
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
        )
    ])