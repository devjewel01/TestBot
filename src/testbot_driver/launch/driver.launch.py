from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('testbot_driver')
    
    # Declare parameters file
    params_file = os.path.join(pkg_dir, 'config', 'hardware_params.yaml')
    
    # Declare launch arguments
    serial_port = LaunchConfiguration('serial_port')
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for motor controller'
    )

    # Configure the hardware interface node
    hardware_interface_node = Node(
        package='testbot_driver',
        executable='hardware_interface',
        name='hardware_interface',
        parameters=[params_file],
        remappings=[
            # Add any topic remappings if needed
            ('wheel_cmd', 'cmd_vel'),
        ],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,

        # Nodes
        hardware_interface_node,
    ])