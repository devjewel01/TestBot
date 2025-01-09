from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('testbot_control')
    
    # Configuration file paths
    control_params = os.path.join(pkg_dir, 'config', 'control_params.yaml')
    
    return LaunchDescription([
        # Launch differential drive controller
        Node(
            package='testbot_control',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen',
            parameters=[control_params],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('left_wheel_velocity', '/left_wheel_velocity'),
                ('right_wheel_velocity', '/right_wheel_velocity'),
            ]
        ),
        
        # Launch odometry publisher
        Node(
            package='testbot_control',
            executable='odometry_publisher',
            name='odometry_publisher',
            output='screen',
            parameters=[control_params],
            remappings=[
                ('left_ticks', '/left_ticks'),
                ('right_ticks', '/right_ticks'),
                ('odom', '/odom')
            ]
        )
    ])