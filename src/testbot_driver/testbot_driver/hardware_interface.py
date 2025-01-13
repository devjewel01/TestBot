#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from .serial_protocol import SerialProtocol
import math
import threading

class HardwareInterface(Node):
    """Hardware interface node that handles communication with the robot's hardware."""
    
    def __init__(self):
        super().__init__('hardware_interface')
        
        self._init_parameters()
        
        self._init_hardware()
        
        self._init_pubs_subs()
        
        self._init_timers()
        
        self.get_logger().info('Hardware interface initialized')

    def _init_parameters(self):
        """Initialize and load parameters."""
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
        
        # Load parameters
        self.params = {
            'serial_port': self.get_parameter('serial_port').value,
            'baudrate': self.get_parameter('baudrate').value,
            'timeout': self.get_parameter('timeout').value,
            'wheel_radius': self.get_parameter('wheel_radius').value,
            'max_motor_rpm': self.get_parameter('max_motor_rpm').value,
            'command_timeout': self.get_parameter('command_timeout').value
        }
        
        # Calculate derived parameters
        self.max_wheel_vel = (self.params['max_motor_rpm'] * 2.0 * math.pi) / 60.0

    def _init_hardware(self):
        """Initialize hardware communication."""
        # Initialize serial connection with thread lock
        self.cmd_lock = threading.Lock()
        self.serial_protocol = SerialProtocol(
            port=self.params['serial_port'],
            baudrate=self.params['baudrate'],
            timeout=self.params['timeout']
        )
        
        if not self.serial_protocol.connect():
            self.get_logger().error('Failed to connect to serial port')
            raise RuntimeError('Failed to connect to motor controller')

        # Register callbacks
        self.serial_protocol.register_encoder_callback(self._encoder_callback)
        self.serial_protocol.register_error_callback(self._error_callback)
        self.serial_protocol.register_status_callback(self._status_callback)

        # Initialize command timing
        self.last_cmd_time = self.get_clock().now()

    def _init_pubs_subs(self):
        """Initialize publishers and subscribers."""
        # Publishers
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
        
        # Subscribers
        self.wheel_cmd_sub = self.create_subscription(
            Twist,
            'wheel_cmd',
            self._wheel_cmd_callback,
            10
        )

    def _init_timers(self):
        """Initialize timers."""
        self.create_timer(0.1, self._watchdog_callback)

    def _wheel_cmd_callback(self, msg: Twist):
        """Handle incoming wheel velocity commands."""
        with self.cmd_lock:
            self.last_cmd_time = self.get_clock().now()
            
            # Extract wheel velocities from Twist message
            left_vel = msg.linear.x   # Left wheel velocity in rad/s
            right_vel = msg.linear.y  # Right wheel velocity in rad/s
            
            # Convert to motor commands and send
            left_pwm, right_pwm = self._convert_velocity_to_pwm(left_vel, right_vel)
            self.serial_protocol.send_motor_command(left_pwm, right_pwm)

    def _convert_velocity_to_pwm(self, left_vel: float, right_vel: float):
        """Convert wheel velocities (rad/s) to PWM commands (-255 to 255)."""
        # Calculate maximum wheel velocity in rad/s
        max_wheel_vel = (self.params['max_motor_rpm'] * 2.0 * math.pi) / 60.0
        
        # Convert to PWM values (-255 to 255)
        left_pwm = int((left_vel / max_wheel_vel) * 255.0)
        right_pwm = int((right_vel / max_wheel_vel) * 255.0)
        
        # Clamp values while preserving direction
        left_pwm = max(min(left_pwm, 255), -255)
        right_pwm = max(min(right_pwm, 255), -255)
        
        return left_pwm, right_pwm

    def _encoder_callback(self, left_ticks: int, right_ticks: int):
        """Handle encoder feedback."""
        # Create messages
        left_msg = Float32()
        right_msg = Float32()
        
        left_msg.data = float(left_ticks)
        right_msg.data = float(right_ticks)
        
        # Publish encoder data
        self.left_encoder_pub.publish(left_msg)
        self.right_encoder_pub.publish(right_msg)

    def _error_callback(self, error_msg: str):
        """Handle error messages."""
        self.get_logger().error(f"Hardware error: {error_msg}")

    def _status_callback(self, status_msg: str):
        """Handle status messages."""
        self.get_logger().info(f"Hardware status: {status_msg}")

    def _watchdog_callback(self):
        """Monitor command timeout."""
        current_time = self.get_clock().now()
        with self.cmd_lock:
            if (current_time - self.last_cmd_time).nanoseconds * 1e-9 > self.params['command_timeout']:
                self.serial_protocol.send_motor_command(0, 0)

    def cleanup(self):
        """Clean up resources before shutdown."""
        if hasattr(self, 'serial_protocol'):
            with self.cmd_lock:
                self.serial_protocol.send_motor_command(0, 0)
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