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
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize hardware interface
        self._init_hardware()
        
        # Initialize publishers and subscribers
        self._init_pubs_subs()
        
        # Initialize timers
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
            
            # Extract wheel velocities
            left_vel = msg.linear.x
            right_vel = msg.linear.y
            
            # Convert to motor commands
            left_cmd, right_cmd = self._convert_velocity_to_pwm(left_vel, right_vel)
            
            # Send commands to motors
            self.serial_protocol.send_motor_command(left_cmd, right_cmd)

    def _convert_velocity_to_pwm(self, left_vel: float, right_vel: float):
        """Convert wheel velocities to PWM commands."""
        # Calculate PWM values normalized to max velocity
        left_pwm = int(abs(left_vel) * 255.0 / self.max_wheel_vel)
        right_pwm = int(abs(right_vel) * 255.0 / self.max_wheel_vel)
        
        # Clamp values
        left_pwm = min(max(left_pwm, 0), 255)
        right_pwm = min(max(right_pwm, 0), 255)
        
        # Apply direction
        left_pwm = -left_pwm if left_vel < 0 else left_pwm
        right_pwm = -right_pwm if right_vel < 0 else right_pwm
        
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