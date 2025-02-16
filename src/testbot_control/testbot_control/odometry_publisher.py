#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from testbot_msgs.msg import EncoderStamped
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
                ('wheel_left_joint', 'left_wheel_joint'),
                ('wheel_right_joint', 'right_wheel_joint'),
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
        self.last_left_encoder = 0.0
        self.last_right_encoder = 0.0
        self.first_reading = True
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Create subscribers for encoder data using custom message type
        self.left_encoder_sub = self.create_subscription(
            EncoderStamped,
            'wheel_encoder/left',
            self.left_encoder_callback,
            10
        )
        self.right_encoder_sub = self.create_subscription(
            EncoderStamped,
            'wheel_encoder/right',
            self.right_encoder_callback,
            10
        )
        
        # Initialize encoder readings
        self.current_left_encoder = 0.0
        self.current_right_encoder = 0.0
        self.current_left_velocity = 0.0
        self.current_right_velocity = 0.0
        self.current_left_position = 0.0
        self.current_right_position = 0.0
        self.left_updated = False
        self.right_updated = False
        
        # Time tracking
        self.last_time = self.get_clock().now()
        
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
        self.wheel_left_joint = self.get_parameter('wheel_left_joint').value
        self.wheel_right_joint = self.get_parameter('wheel_right_joint').value
        self.pose_covariance_diagonal = self.get_parameter('pose_covariance_diagonal').value
        self.twist_covariance_diagonal = self.get_parameter('twist_covariance_diagonal').value
        
        # Calculate meters per encoder tick
        self.meters_per_tick = (2 * math.pi * self.wheel_radius) / self.encoder_resolution

    def left_encoder_callback(self, msg: EncoderStamped):
        """Handle left wheel encoder updates."""
        self.current_left_encoder = float(msg.left_ticks)
        self.current_left_velocity = msg.left_velocity
        self.current_left_position = msg.left_position
        self.left_updated = True

    def right_encoder_callback(self, msg: EncoderStamped):
        """Handle right wheel encoder updates."""
        self.current_right_encoder = float(msg.right_ticks)
        self.current_right_velocity = msg.right_velocity
        self.current_right_position = msg.right_position
        self.right_updated = True

    def timer_callback(self):
        """Calculate and publish odometry at regular intervals."""
        if not (self.left_updated and self.right_updated):
            return
            
        # Reset update flags
        self.left_updated = False
        self.right_updated = False
        
        # Skip first reading to establish baseline
        if self.first_reading:
            self.last_left_encoder = self.current_left_encoder
            self.last_right_encoder = self.current_right_encoder
            self.first_reading = False
            self.last_time = self.get_clock().now()
            return
        
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Calculate encoder differences
        left_ticks = self.current_left_encoder - self.last_left_encoder
        right_ticks = self.current_right_encoder - self.last_right_encoder
        
        # Update stored encoder values
        self.last_left_encoder = self.current_left_encoder
        self.last_right_encoder = self.current_right_encoder
        
        # Calculate distance traveled by each wheel
        left_distance = left_ticks * self.meters_per_tick
        right_distance = right_ticks * self.meters_per_tick
        
        # Calculate robot movement
        distance = (left_distance + right_distance) / 2.0
        rotation = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot pose
        if abs(rotation) < 1e-6:
            # Straight line movement
            self.x += distance * math.cos(self.theta)
            self.y += distance * math.sin(self.theta)
        else:
            # Arc movement
            radius = distance / rotation
            self.x += radius * (math.sin(self.theta + rotation) - math.sin(self.theta))
            self.y -= radius * (math.cos(self.theta + rotation) - math.cos(self.theta))
        
        self.theta = self.normalize_angle(self.theta + rotation)
        
        # Calculate velocities
        linear_velocity = distance / dt if dt > 0 else 0.0
        angular_velocity = rotation / dt if dt > 0 else 0.0
        
        # Publish transform if enabled
        if self.publish_tf:
            self._publish_tf(current_time)
        
        # Publish odometry message
        self._publish_odom(current_time, linear_velocity, angular_velocity)
        
        # Publish joint states
        self._publish_joint_states(current_time)

    def _publish_tf(self, timestamp):
        """Publish transform from odom to base_footprint."""
        t = TransformStamped()
        
        t.header.stamp = timestamp.to_msg()
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

    def _publish_odom(self, timestamp, linear_vel, angular_vel):
        """Publish odometry message."""
        odom = Odometry()
        
        # Set header
        odom.header.stamp = timestamp.to_msg()
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
        
        # Set covariances using the configured values
        odom.pose.covariance = self._create_diagonal_matrix(self.pose_covariance_diagonal)
        odom.twist.covariance = self._create_diagonal_matrix(self.twist_covariance_diagonal)
        
        self.odom_pub.publish(odom)

    def _publish_joint_states(self, timestamp):
        """Publish joint states for wheels."""
        joint_state = JointState()
        
        joint_state.header.stamp = timestamp.to_msg()
        joint_state.name = [self.wheel_left_joint, self.wheel_right_joint]
        
        # Use the processed position and velocity from encoder messages
        joint_state.position = [self.current_left_position, self.current_right_position]
        joint_state.velocity = [self.current_left_velocity, self.current_right_velocity]
        joint_state.effort = [0.0, 0.0]  # Could add motor efforts if available
        
        self.joint_pub.publish(joint_state)

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

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

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