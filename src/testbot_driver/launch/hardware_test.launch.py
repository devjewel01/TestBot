from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('testbot_driver')
    
    # Declare parameters
    config_file = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')
    
    return LaunchDescription([
        # Robot state publisher (load URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', os.path.join(
                        get_package_share_directory('testbot_description'),
                        'urdf', 'testbot.urdf.xacro'
                    )
                ])
            }]
        ),
        
        # Hardware interface node
        Node(
            package='testbot_driver',
            executable='hardware_interface',
            name='hardware_interface',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('odom', '/odom'),
            ]
        )
    ])