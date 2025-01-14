#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from testbot_msgs.msg import MotorCommand, MotorFeedback, EncoderStamped, MotorStatus
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
                ('max_motor_rpm', 500.0),
                ('command_timeout', 0.5),
                ('encoder_resolution', 1440)  # Added encoder resolution parameter
            ]
        )
        
        # Load parameters
        self.params = {
            'serial_port': self.get_parameter('serial_port').value,
            'baudrate': self.get_parameter('baudrate').value,
            'timeout': self.get_parameter('timeout').value,
            'wheel_radius': self.get_parameter('wheel_radius').value,
            'max_motor_rpm': self.get_parameter('max_motor_rpm').value,
            'command_timeout': self.get_parameter('command_timeout').value,
            'encoder_resolution': self.get_parameter('encoder_resolution').value
        }
        
        # Calculate derived parameters
        self.max_wheel_vel = (self.params['max_motor_rpm'] * 2.0 * math.pi) / 60.0
        self.radians_per_tick = (2.0 * math.pi) / self.params['encoder_resolution']

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

        # Initialize command timing and motor state
        self.last_cmd_time = self.get_clock().now()
        self.last_encoder_time = self.get_clock().now()
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.motor_status = MotorStatus.MOTOR_OK

    def _init_pubs_subs(self):
        """Initialize publishers and subscribers."""
        # Publishers
        self.left_encoder_pub = self.create_publisher(
            EncoderStamped, 
            'wheel_encoder/left', 
            10
        )
        self.right_encoder_pub = self.create_publisher(
            EncoderStamped, 
            'wheel_encoder/right', 
            10
        )
        self.motor_feedback_pub = self.create_publisher(
            MotorFeedback,
            'motor_feedback',
            10
        )
        self.motor_status_pub = self.create_publisher(
            MotorStatus,
            'motor_status',
            10
        )
        
        # Subscribers
        self.motor_cmd_sub = self.create_subscription(
            MotorCommand,
            'motor_command',
            self._motor_cmd_callback,
            10
        )

    def _init_timers(self):
        """Initialize timers."""
        self.create_timer(0.1, self._watchdog_callback)
        self.create_timer(1.0, self._status_publisher_callback)

    def _motor_cmd_callback(self, msg: MotorCommand):
        """Handle incoming motor commands."""
        with self.cmd_lock:
            self.last_cmd_time = self.get_clock().now()
            
            if msg.emergency_stop:
                # Handle emergency stop
                self.serial_protocol.send_motor_command(0, 0)
                return
            
            # Convert velocities to PWM
            left_pwm, right_pwm = self._convert_velocity_to_pwm(
                msg.left_motor_vel,
                msg.right_motor_vel
            )
            
            # Send commands to hardware
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
        """Handle encoder feedback and publish comprehensive encoder data."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_encoder_time).nanoseconds / 1e9
        
        # Calculate velocities (rad/s)
        if dt > 0:
            left_diff_ticks = left_ticks - self.last_left_ticks
            right_diff_ticks = right_ticks - self.last_right_ticks
            
            left_velocity = (left_diff_ticks * self.radians_per_tick) / dt
            right_velocity = (right_diff_ticks * self.radians_per_tick) / dt
        else:
            left_velocity = 0.0
            right_velocity = 0.0

        # Calculate positions (radians)
        left_position = left_ticks * self.radians_per_tick
        right_position = right_ticks * self.radians_per_tick
        
        # Create and publish left encoder message
        left_msg = EncoderStamped()
        left_msg.header.stamp = current_time.to_msg()
        left_msg.left_ticks = left_ticks
        left_msg.left_velocity = left_velocity
        left_msg.left_position = left_position
        self.left_encoder_pub.publish(left_msg)
        
        # Create and publish right encoder message
        right_msg = EncoderStamped()
        right_msg.header.stamp = current_time.to_msg()
        right_msg.right_ticks = right_ticks
        right_msg.right_velocity = right_velocity
        right_msg.right_position = right_position
        self.right_encoder_pub.publish(right_msg)
        
        # Create and publish motor feedback
        feedback_msg = MotorFeedback()
        feedback_msg.header.stamp = current_time.to_msg()
        feedback_msg.left_motor_vel = left_velocity
        feedback_msg.right_motor_vel = right_velocity
        feedback_msg.left_duty = 0.0  # Could calculate from PWM if available
        feedback_msg.right_duty = 0.0  # Could calculate from PWM if available
        self.motor_feedback_pub.publish(feedback_msg)
        
        # Update stored values
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_encoder_time = current_time

    def _error_callback(self, error_msg: str):
        """Handle error messages and update motor status."""
        self.get_logger().error(f"Hardware error: {error_msg}")
        self.motor_status = MotorStatus.MOTOR_ERROR
        self._publish_motor_status(error_msg)

    def _status_callback(self, status_msg: str):
        """Handle status messages."""
        self.get_logger().info(f"Hardware status: {status_msg}")
        self._publish_motor_status(status_msg)

    def _status_publisher_callback(self):
        """Periodically publish motor status."""
        self._publish_motor_status("Normal operation")

    def _publish_motor_status(self, message: str):
        """Publish motor status message."""
        status_msg = MotorStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.left_motor_status = self.motor_status
        status_msg.right_motor_status = self.motor_status
        status_msg.motor_voltage = 0.0  # Could add voltage monitoring
        status_msg.motor_temperature = 0.0  # Could add temperature monitoring
        status_msg.status_message = message
        self.motor_status_pub.publish(status_msg)

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