from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('testbot_driver')
    
    params_file = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')

    hardware_interface_node = Node(
        package='testbot_driver',
        executable='hardware_interface',
        name='hardware_interface',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        hardware_interface_node,
    ])