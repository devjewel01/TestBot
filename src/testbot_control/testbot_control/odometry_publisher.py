#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import numpy as np

class OdometryPublisher(Node):
    """Publishes odometry information based on wheel encoder data."""
    
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.022),
                ('wheel_separation', 0.13),
                ('encoder_resolution', 1440),
                ('publish_frequency', 50.0),
                ('publish_tf', True),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_footprint'),
                ('pose_covariance_diagonal', [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]),
                ('twist_covariance_diagonal', [0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
            ]
        )
        
        # Load parameters
        self._load_parameters()
        
        # Initialize odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.first_reading = True
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create subscribers
        self.left_ticks_sub = self.create_subscription(
            Float32,
            'left_ticks',
            self.left_ticks_callback,
            10
        )
        self.right_ticks_sub = self.create_subscription(
            Float32,
            'right_ticks',
            self.right_ticks_callback,
            10
        )
        
        # Initialize variables for encoder readings
        self.current_left_ticks = 0
        self.current_right_ticks = 0
        self.left_ticks_updated = False
        self.right_ticks_updated = False
        
        # Create timer for publishing
        self.create_timer(1.0/self.publish_frequency, self.timer_callback)
        
        self.get_logger().info('Odometry publisher initialized')

    def _load_parameters(self):
        """Load parameters from ROS parameters."""
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal').value
        self.twist_covariance_diagonal = self.get_parameter('twist_covariance_diagonal').value
        
        # Calculate meters per tick
        self.meters_per_tick = (2 * math.pi * self.wheel_radius) / self.encoder_resolution

    def left_ticks_callback(self, msg):
        """Handle left wheel encoder ticks."""
        self.current_left_ticks = msg.data
        self.left_ticks_updated = True

    def right_ticks_callback(self, msg):
        """Handle right wheel encoder ticks."""
        self.current_right_ticks = msg.data
        self.right_ticks_updated = True

    def timer_callback(self):
        """Calculate and publish odometry at regular intervals."""
        if not (self.left_ticks_updated and self.right_ticks_updated):
            return
            
        # Reset update flags
        self.left_ticks_updated = False
        self.right_ticks_updated = False
        
        # Skip first reading to establish baseline
        if self.first_reading:
            self.last_left_ticks = self.current_left_ticks
            self.last_right_ticks = self.current_right_ticks
            self.first_reading = False
            return
        
        # Calculate wheel rotations
        left_ticks_diff = self.current_left_ticks - self.last_left_ticks
        right_ticks_diff = self.current_right_ticks - self.last_right_ticks
        
        # Update stored ticks
        self.last_left_ticks = self.current_left_ticks
        self.last_right_ticks = self.current_right_ticks
        
        # Calculate distance traveled by each wheel
        left_distance = left_ticks_diff * self.meters_per_tick
        right_distance = right_ticks_diff * self.meters_per_tick
        
        # Calculate robot's movement
        center_distance = (left_distance + right_distance) / 2.0
        theta_change = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot's pose
        if abs(theta_change) < 1e-6:
            # Straight line movement
            self.x += center_distance * math.cos(self.theta)
            self.y += center_distance * math.sin(self.theta)
        else:
            # Arc movement
            radius = center_distance / theta_change
            self.x += radius * (math.sin(self.theta + theta_change) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + theta_change) - math.cos(self.theta))
        
        self.theta += theta_change
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize angle
        
        # Calculate velocities
        dt = 1.0 / self.publish_frequency
        linear_velocity = center_distance / dt
        angular_velocity = theta_change / dt
        
        # Publish transform if enabled
        if self.publish_tf:
            self._publish_tf()
        
        # Publish odometry message
        self._publish_odom(linear_velocity, angular_velocity)

    def _publish_tf(self):
        """Publish transform from odom to base_footprint."""
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        q = self._euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def _publish_odom(self, linear_vel, angular_vel):
        """Publish odometry message."""
        odom = Odometry()
        
        # Set header
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        q = self._euler_to_quaternion(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Set velocities
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        # Set covariances
        odom.pose.covariance = self._create_diagonal_matrix(self.pose_covariance_diagonal)
        odom.twist.covariance = self._create_diagonal_matrix(self.twist_covariance_diagonal)
        
        self.odom_pub.publish(odom)

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
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

    @staticmethod
    def _create_diagonal_matrix(diagonal):
        """Create 6x6 diagonal covariance matrix from diagonal elements."""
        matrix = [0.0] * 36
        for i in range(6):
            matrix[i * 7] = diagonal[i]
        return matrix

def main(args=None):
    rclpy.init(args=args)
    
    try:
        odom_publisher = OdometryPublisher()
        rclpy.spin(odom_publisher)
    except Exception as e:
        print(f"Error in odometry publisher: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()