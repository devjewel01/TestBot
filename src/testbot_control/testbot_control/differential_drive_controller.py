#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import numpy as np

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.022),
                ('wheel_separation', 0.13),
                ('max_linear_speed', 0.2),
                ('max_angular_speed', 1.0),
                ('min_linear_speed', 0.02),
                ('min_angular_speed', 0.05),
                ('linear_acceleration', 0.2),
                ('angular_acceleration', 0.4),
                ('control_frequency', 50.0),
                ('cmd_vel_timeout', 0.5),
                ('velocity_smoothing', True),
                ('smoothing_factor', 0.2)
            ]
        )
        
        # Load parameters
        self._load_parameters()
        
        # Initialize velocities
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.new_command_received = False  # Flag to track new commands
        
        # Publishers
        self.wheel_cmd_pub = self.create_publisher(
            Twist,
            'wheel_cmd',
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Control loop timer
        self.control_timer = self.create_timer(
            1.0/self.control_frequency,
            self.control_loop
        )
        
        self.get_logger().info('Differential drive controller initialized')

    def _load_parameters(self):
        """Load parameters from ROS parameters."""
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.linear_acceleration = self.get_parameter('linear_acceleration').value
        self.angular_acceleration = self.get_parameter('angular_acceleration').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.velocity_smoothing = self.get_parameter('velocity_smoothing').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        
        self.dt = 1.0 / self.control_frequency
        self.max_wheel_speed = self.max_linear_speed / self.wheel_radius

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands."""
        self.new_command_received = True  # Set flag for new command
        
        # Update target velocities with limits
        self.target_linear_vel = self._clamp(
            msg.linear.x,
            -self.max_linear_speed,
            self.max_linear_speed
        )
        self.target_angular_vel = self._clamp(
            msg.angular.z,
            -self.max_angular_speed,
            self.max_angular_speed
        )
        
        # Update command time
        self.last_cmd_time = self.get_clock().now()
        
        # Process and publish immediately for responsive control
        self.process_and_publish_command()

    def process_and_publish_command(self):
        """Process current command and publish wheel velocities."""
        # Update current velocities
        self.current_linear_vel = self._update_velocity(
            self.current_linear_vel,
            self.target_linear_vel,
            self.linear_acceleration
        )
        
        self.current_angular_vel = self._update_velocity(
            self.current_angular_vel,
            self.target_angular_vel,
            self.angular_acceleration
        )
        
        # Calculate and publish wheel velocities
        left_wheel_vel, right_wheel_vel = self._calculate_wheel_velocities(
            self.current_linear_vel,
            self.current_angular_vel
        )
        
        if self.current_linear_vel != 0.0 or self.current_angular_vel != 0.0:
            self._publish_wheel_velocities(left_wheel_vel, right_wheel_vel)

    def control_loop(self):
        """Main control loop for monitoring command timeout."""
        current_time = self.get_clock().now()

        # Check for command timeout
        if (current_time - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout:
            if self.target_linear_vel != 0.0 or self.target_angular_vel != 0.0:
                self.target_linear_vel = 0.0
                self.target_angular_vel = 0.0
                self.current_linear_vel = 0.0
                self.current_angular_vel = 0.0
                self._publish_wheel_velocities(0.0, 0.0)  # Publish stop command
                self.get_logger().debug('Command timeout - stopping robot')

    def _update_velocity(self, current: float, target: float, max_accel: float) -> float:
        """Update velocity considering acceleration limits and smoothing."""
        if abs(target) < self.min_linear_speed and abs(target - current) < 0.001:
            return 0.0
            
        if self.velocity_smoothing:
            smoothing = self.smoothing_factor * 2.0 if target == 0 else self.smoothing_factor
            new_vel = current + smoothing * (target - current)
        else:
            max_change = max_accel * self.dt
            if target > current:
                new_vel = min(target, current + max_change)
            else:
                new_vel = max(target, current - max_change)
            
        if abs(new_vel) < self.min_linear_speed:
            new_vel = 0.0
        
        return new_vel

    def _calculate_wheel_velocities(self, linear_vel: float, angular_vel: float) -> tuple:
        """Convert linear and angular velocities to wheel velocities."""
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        scale = abs(max(left_wheel_vel, right_wheel_vel, key=abs)) / self.max_wheel_speed
        if scale > 1.0:
            left_wheel_vel /= scale
            right_wheel_vel /= scale
        
        return left_wheel_vel, right_wheel_vel

    def _publish_wheel_velocities(self, left_vel: float, right_vel: float):
        """Publish wheel velocities using Twist message."""
        wheel_cmd = Twist()
        wheel_cmd.linear.x = float(left_vel)
        wheel_cmd.linear.y = float(right_vel)
        wheel_cmd.linear.z = 0.0
        wheel_cmd.angular.x = 0.0
        wheel_cmd.angular.y = 0.0
        wheel_cmd.angular.z = 0.0
        
        self.wheel_cmd_pub.publish(wheel_cmd)

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        """Clamp value between min and max."""
        return max(min(value, max_value), min_value)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = DifferentialDriveController()
        rclpy.spin(controller)
    except Exception as e:
        print(f"Error in differential drive controller: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()