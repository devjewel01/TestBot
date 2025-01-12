from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_testbot_control = get_package_share_directory('testbot_control')
    
    # Get the control config file path
    config_file = os.path.join(
        pkg_testbot_control,
        'config',
        'control_params.yaml'
    )
    
    # Create nodes
    differential_drive_node = Node(
        package='testbot_control',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[config_file]
    )
    
    odometry_node = Node(
        package='testbot_control',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[config_file]
    )
    
    # Create and return launch description
    return LaunchDescription([
        differential_drive_node,
        odometry_node
    ])