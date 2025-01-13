from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    testbot_description_pkg = get_package_share_directory('testbot_description')
    testbot_driver_pkg = get_package_share_directory('testbot_driver')
    testbot_control_pkg = get_package_share_directory('testbot_control')

    urdf_file = os.path.join(testbot_description_pkg, 'urdf', 'testbot.urdf.xacro')
    hardware_config = os.path.join(testbot_driver_pkg, 'config', 'hardware_params.yaml')
    control_config = os.path.join(testbot_control_pkg, 'config', 'control_params.yaml')


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([FindExecutable(name='xacro'), ' ', urdf_file]),
                value_type=str
            ),
        }]
    )

    hardware_interface_node = Node(
        package='testbot_driver',
        executable='hardware_interface',
        name='hardware_interface',
        parameters=[hardware_config],
        output='screen'
    )

    differential_drive_node = Node(
        package='testbot_control',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        parameters=[control_config],
        output='screen'
    )

    odometry_node = Node(
        package='testbot_control',
        executable='odometry_publisher',
        name='odometry_publisher',
        parameters=[control_config],
        output='screen'
    )

    rviz_config = os.path.join(testbot_description_pkg, 'rviz', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([        
        hardware_interface_node,
        differential_drive_node,
        odometry_node,
        robot_state_publisher_node,
        rviz_node
    ])