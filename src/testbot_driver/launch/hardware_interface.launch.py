from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare package paths
    pkg_testbot_driver = FindPackageShare('testbot_driver')
    pkg_testbot_description = FindPackageShare('testbot_description')
    
    # Create launch configuration variables
    hardware_config = PathJoinSubstitution([pkg_testbot_driver, 'config', 'hardware_params.yaml'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32 connection'
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate for serial communication'
        )
    ]

    # Robot state publisher for TF
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', 
                PathJoinSubstitution([pkg_testbot_description, 'urdf', 'testbot.urdf.xacro'])
            ]),
            'use_sim_time': use_sim_time
        }]
    )

    # Hardware interface node
    hardware_interface_node = Node(
        package='testbot_driver',
        executable='hardware_interface',
        name='hardware_interface',
        output='screen',
        parameters=[
            hardware_config,
            {
                'use_sim_time': use_sim_time,
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate')
            }
        ]
    )

    # Joint state publisher
    joint_state_pub_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Add event handler to ensure robot_state_publisher starts after joint_state_publisher
    event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_pub_node,
            on_start=[robot_state_pub_node]
        )
    )

    # Return the launch description
    return LaunchDescription(
        launch_args + [
            joint_state_pub_node,
            event_handler,
            hardware_interface_node
        ]
    )