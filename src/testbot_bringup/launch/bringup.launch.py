from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    RegisterEventHandler
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package share directories
    pkg_testbot_description = FindPackageShare('testbot_description')
    pkg_testbot_driver = FindPackageShare('testbot_driver')
    pkg_testbot_control = FindPackageShare('testbot_control')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Path to URDF and RViz config
    urdf_path = PathJoinSubstitution([pkg_testbot_description, 'urdf', 'testbot.urdf.xacro'])
    rviz_config_path = PathJoinSubstitution([pkg_testbot_description, 'rviz', 'view_robot.rviz'])

    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2'
        )
    ]

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # Include Hardware Interface
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

    # Include Control Launch
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_testbot_control,
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription(declare_args + [
        # Start joint state publisher first
        joint_state_publisher,
        
        # Start robot state publisher after joint state publisher
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=joint_state_publisher,
                on_start=[robot_state_publisher]
            )
        ),
        
        # Launch hardware interface
        hardware_launch,
        
        # Launch control system
        control_launch,
        
        # Launch RViz last
        rviz_node
    ])