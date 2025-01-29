# joystick_teleop.launch.py
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

        # Teleop twist converter with PS5 mappings
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[{
                # Forward/backward - Left stick up/down
                'axis_linear.x': 1,
                'scale_linear.x': 0.8,     # Normal speed
                'scale_linear_turbo.x': 1.5,  # Turbo speed  
                
                # Left/right turning - Left stick left/right
                'axis_angular.yaw': 0,     # Left stick horizontal
                'scale_angular.yaw': 1.5,  # Normal turn speed
                'scale_angular_turbo.yaw': 2.5,  # Turbo turn speed
                
                # Turbo button configuration
                'enable_button': 7,        # R2 button
                'enable_turbo_button': 7,  # Same as enable_button for consistent turbo
                'require_enable_button': False
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
            ]
        )
    ])