from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device_id': 0,
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Teleop twist converter with working PS5 mappings
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                # Working settings for forward/backward
                'axis_linear.x': 1,        # Left stick up/down
                'scale_linear.x': 0.3,     # Normal speed
                'scale_linear_turbo.x': 0.6,  # Turbo speed
                
                # Updated turning settings
                'axis_angular.yaw': 2,     # Try different axis for turning
                'scale_angular.yaw': 0.5,  # Normal turn speed
                'scale_angular_turbo.yaw': 1.0,  # Turbo turn speed
                
                # Keep working turbo settings
                'enable_turbo_button': 7,  # R2 button
                'require_enable_button': False
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ]
        )
    ])