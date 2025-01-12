from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    pkg_testbot_control = FindPackageShare('testbot_control')
    pkg_testbot_driver = FindPackageShare('testbot_driver')
    
    # Launch configurations
    control_config = PathJoinSubstitution([pkg_testbot_control, 'config', 'control_params.yaml'])
    
    # Create launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_teleop = LaunchConfiguration('start_teleop')
    teleop_type = LaunchConfiguration('teleop_type')
    
    # Declare launch arguments first
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'start_teleop',
            default_value='false',
            description='Start teleoperation node'
        ),
        DeclareLaunchArgument(
            'teleop_type',
            default_value='none',
            description='Type of teleoperation (keyboard or joystick)'
        )
    ]

    # Include hardware interface launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_testbot_driver,
                'launch',
                'hardware_interface.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Differential drive controller node
    diff_drive_node = Node(
        package='testbot_control',
        executable='differential_drive_controller',
        name='differential_drive_controller',
        output='screen',
        parameters=[
            control_config,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Odometry publisher node
    odom_node = Node(
        package='testbot_control',
        executable='odometry_publisher',
        name='odometry_publisher',
        output='screen',
        parameters=[
            control_config,
            {
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Return the launch description with all components
    return LaunchDescription(
        declare_args + [
            hardware_launch,
            diff_drive_node,
            odom_node
        ]
    )