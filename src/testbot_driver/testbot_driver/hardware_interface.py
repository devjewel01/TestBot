#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from .serial_protocol import SerialProtocol
import math

class HardwareInterface(Node):
    """Hardware interface node that handles communication with the robot's hardware."""
    
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
                ('max_motor_rpm', 100.0),
                ('command_timeout', 0.5)
            ]
        )
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.timeout = self.get_parameter('timeout').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_motor_rpm = self.get_parameter('max_motor_rpm').value
        self.command_timeout = self.get_parameter('command_timeout').value

        # Initialize serial connection
        self.serial_protocol = SerialProtocol(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )
        
        if not self.serial_protocol.connect():
            self.get_logger().error('Failed to connect to serial port')
            return

        # Register callbacks for serial communication
        self.serial_protocol.register_encoder_callback(self.encoder_callback)
        self.serial_protocol.register_error_callback(self.error_callback)
        self.serial_protocol.register_status_callback(self.status_callback)

        # Command timeout tracking
        self.last_cmd_time = self.get_clock().now()
        
        # Publishers for encoder data
        self.left_encoder_pub = self.create_publisher(
            Float32, 
            'wheel_encoder/left', 
            10
        )
        self.right_encoder_pub = self.create_publisher(
            Float32, 
            'wheel_encoder/right', 
            10
        )
        
        # Subscriber for wheel velocity commands
        self.wheel_cmd_sub = self.create_subscription(
            Twist,  # Using Twist for wheel velocities (linear.x = left, linear.y = right)
            'wheel_cmd',
            self.wheel_cmd_callback,
            10
        )
        
        # Timer for checking command timeout
        self.create_timer(0.1, self.check_timeout)
        
        self.get_logger().info('Hardware interface initialized')

    def wheel_cmd_callback(self, msg: Twist):
        """Handle incoming wheel velocity commands."""
        self.last_cmd_time = self.get_clock().now()
        
        # Extract left and right wheel velocities from Twist message
        left_wheel_vel = msg.linear.x   # Left wheel velocity in rad/s
        right_wheel_vel = msg.linear.y  # Right wheel velocity in rad/s
        
        # Convert wheel velocities to PWM values (0-255)
        # First normalize to max wheel velocity
        max_wheel_vel = (self.max_motor_rpm * 2.0 * math.pi) / 60.0  # Convert max RPM to rad/s
        
        # Calculate PWM values
        left_pwm = int(abs(left_wheel_vel) * 255.0 / max_wheel_vel)
        right_pwm = int(abs(right_wheel_vel) * 255.0 / max_wheel_vel)
        
        # Clamp PWM values
        left_pwm = min(max(left_pwm, 0), 255)
        right_pwm = min(max(right_pwm, 0), 255)
        
        # Set direction based on velocity sign
        left_pwm = -left_pwm if left_wheel_vel < 0 else left_pwm
        right_pwm = -right_pwm if right_wheel_vel < 0 else right_pwm
        
        # Send commands to motors
        self.serial_protocol.send_motor_command(left_pwm, right_pwm)

    def encoder_callback(self, left_ticks: int, right_ticks: int):
        """Publish encoder data from hardware."""
        # Publish raw encoder ticks
        left_msg = Float32()
        right_msg = Float32()
        
        left_msg.data = float(left_ticks)
        right_msg.data = float(right_ticks)
        
        self.left_encoder_pub.publish(left_msg)
        self.right_encoder_pub.publish(right_msg)

    def error_callback(self, error_msg: str):
        """Handle error messages from serial protocol."""
        self.get_logger().error(f"Hardware error: {error_msg}")

    def status_callback(self, status_msg: str):
        """Handle status messages from serial protocol."""
        self.get_logger().info(f"Hardware status: {status_msg}")

    def check_timeout(self):
        """Check for command timeout and stop motors if necessary."""
        current_time = self.get_clock().now()
        if (current_time - self.last_cmd_time).nanoseconds * 1e-9 > self.command_timeout:
            # Stop motors on timeout
            self.serial_protocol.send_motor_command(0.0, 0.0)

    def cleanup(self):
        """Clean up resources before shutdown."""
        if hasattr(self, 'serial_protocol'):
            # Stop motors
            self.serial_protocol.send_motor_command(0.0, 0.0)
            # Disconnect from serial
            self.serial_protocol.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

      