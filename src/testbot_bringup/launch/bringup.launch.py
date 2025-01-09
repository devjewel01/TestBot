#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package share directories
    pkg_testbot_description = FindPackageShare('testbot_description')
    pkg_testbot_bringup = FindPackageShare('testbot_bringup')

    # Robot description
    robot_description = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([pkg_testbot_description, 'urdf', 'testbot.urdf.xacro'])
        ]
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Hardware driver
        Node(
            package='testbot_driver',
            executable='hardware_interface',
            name='hardware_interface',
            output='screen'
        ),

        # Wait for TF tree to establish
        TimerAction(
            period=2.0,
            actions=[
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
                        'scan_mode': 'Standard'
                    }],
                    output='screen'
                ),
            ]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
    ])