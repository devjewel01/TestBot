#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState 
from std_msgs.msg import Float32
import math
import numpy as np
import time
import threading
from tf2_ros import TransformBroadcaster
from .serial_protocol import SerialProtocol

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('timeout', 0.1),
                ('wheel_radius', 0.022),
                ('wheel_separation', 0.13),
                ('encoder_ticks_per_rev', 1440),
                ('max_linear_speed', 0.5),
                ('max_angular_speed', 2.0),
                ('control_frequency', 50.0),
                ('pose_covariance_diagonal', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
                ('twist_covariance_diagonal', [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
            ]
        )
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal').value
        self.twist_covariance_diagonal = self.get_parameter('twist_covariance_diagonal').value

        # Command timeout handling
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        
        # Initialize serial connection
        self.serial_protocol = SerialProtocol(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        
        if not self.serial_protocol.connect():
            self.get_logger().error('Failed to connect to serial port')
            return

        # Register callbacks
        self.serial_protocol.register_encoder_callback(self.encoder_callback)
        self.serial_protocol.register_error_callback(self.error_callback)
        self.serial_protocol.register_status_callback(self.status_callback)

        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_time = self.get_clock().now()

        # Velocity tracking
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.left_ticks_pub = self.create_publisher(Float32, 'left_ticks', 10)
        self.right_ticks_pub = self.create_publisher(Float32, 'right_ticks', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for main control loop
        self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info('Hardware interface initialized')

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        self.last_cmd_time = self.get_clock().now()

        # Store target velocities
        self.target_linear_x = msg.linear.x
        self.target_angular_z = msg.angular.z
        
        # Apply speed limits
        linear_x = max(min(self.target_linear_x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(self.target_angular_z, self.max_angular_speed), -self.max_angular_speed)
        
        # Calculate wheel velocities
        left_velocity = (linear_x - angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        right_velocity = (linear_x + angular_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Send to motors
        self.serial_protocol.send_motor_command(left_velocity, right_velocity)
    
    def publish_joint_states(self, left_ticks, right_ticks):
        """Publish joint states for wheels"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Calculate wheel positions from encoder ticks
        left_pos = (left_ticks / self.encoder_ticks_per_rev) * 2 * math.pi
        right_pos = (right_ticks / self.encoder_ticks_per_rev) * 2 * math.pi
        
        joint_state.position = [left_pos, right_pos]
        joint_state.velocity = [0.0, 0.0]  # Optional: Add actual velocities if needed
        joint_state.effort = [0.0, 0.0]    # Optional: Add motor efforts if available
        
        self.joint_pub.publish(joint_state)
    
    def encoder_callback(self, left_ticks: int, right_ticks: int):
        """Handle encoder updates from serial protocol"""
        # Publish raw ticks
        self.left_ticks_pub.publish(Float32(data=float(left_ticks)))
        self.right_ticks_pub.publish(Float32(data=float(right_ticks)))
        
        # Update odometry
        self.update_odometry(left_ticks, right_ticks)

        # Add joint state publishing
        self.publish_joint_states(left_ticks, right_ticks)

    def error_callback(self, error_msg: str):
        """Handle error messages from serial protocol"""
        self.get_logger().error(f"Hardware error: {error_msg}")

    def status_callback(self, status_msg: str):
        """Handle status messages from serial protocol"""
        self.get_logger().info(f"Hardware status: {status_msg}")

    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, 'serial_protocol'):
            self.serial_protocol.disconnect()

    def update_odometry(self, left_ticks, right_ticks):
        """Update odometry based on encoder ticks"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate wheel rotations
        left_diff = (left_ticks - self.last_enc_left) / self.encoder_ticks_per_rev
        right_diff = (right_ticks - self.last_enc_right) / self.encoder_ticks_per_rev
        
        # Update last encoder values
        self.last_enc_left = left_ticks
        self.last_enc_right = right_ticks
        
        # Calculate distance traveled by each wheel
        left_dist = left_diff * 2 * math.pi * self.wheel_radius
        right_dist = right_diff * 2 * math.pi * self.wheel_radius
        
        # Calculate robot movement
        center_dist = (left_dist + right_dist) / 2.0
        theta_diff = (right_dist - left_dist) / self.wheel_separation
        
        # Update pose
        self.theta += theta_diff
        self.x += center_dist * math.cos(self.theta)
        self.y += center_dist * math.sin(self.theta)

        # Update velocities
        if dt > 0:
            self.current_linear_x = center_dist / dt
            self.current_angular_z = theta_diff / dt
        
        self.last_time = current_time

    def create_covariance_matrix(self, diagonal):
        """Create 6x6 covariance matrix from diagonal elements"""
        matrix = [0.0] * 36
        for i in range(6):
            matrix[i * 7] = diagonal[i]  # Set diagonal elements
        return matrix

    def control_loop(self):
        """Main control loop"""
        current_time = self.get_clock().now()

        # Check for command timeout
        if (current_time - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.serial_protocol.send_motor_command(0.0, 0.0)
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        quat = self.euler_to_quaternion(0.0, 0.0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Set covariance
        odom.pose.covariance = self.create_covariance_matrix(self.pose_covariance_diagonal)
        odom.twist.covariance = self.create_covariance_matrix(self.twist_covariance_diagonal)

        # Set velocity
        odom.twist.twist.linear.x = self.current_linear_x
        odom.twist.twist.angular.z = self.current_angular_z
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        self.broadcast_transform(current_time)

    def broadcast_transform(self, current_time):
        """Broadcast TF transform for odometry"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        quat = self.euler_to_quaternion(0.0, 0.0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        q[3] = cr * cp * cy + sr * sp * sy  # w

        return q

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if hasattr(node, 'serial_protocol'):
            node.serial_protocol.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()