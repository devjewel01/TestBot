#!/usr/bin/env python3
import serial
import threading
import time
import logging
from typing import Optional, Tuple, Callable

class SerialProtocol:
    """Handles serial communication with the ESP32 motor controller"""
    
    # Message Types
    MSG_MOTOR = 'M'  # Motor commands
    MSG_ENCODER = 'E'  # Encoder data
    MSG_STATUS = 'S'  # Status messages
    MSG_ERROR = 'X'  # Error messages
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """Initialize serial communication"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.running = False
        self.lock = threading.Lock()
        self.logger = logging.getLogger("SerialProtocol")
        
        # Callback functions
        self.encoder_callback: Optional[Callable[[int, int], None]] = None
        self.status_callback: Optional[Callable[[str], None]] = None
        self.error_callback: Optional[Callable[[str], None]] = None

    def connect(self) -> bool:
        """Establish serial connection"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self.running = True
            
            # Start reading thread
            self.read_thread = threading.Thread(target=self._read_loop)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            self.logger.info(f"Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            self.logger.error(f"Failed to connect to {self.port}: {str(e)}")
            return False

    def disconnect(self):
        """Close serial connection"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            with self.lock:
                self.serial_conn.close()
        self.logger.info("Disconnected from serial port")

    def send_motor_command(self, left_speed: float, right_speed: float) -> bool:
        """
        Send motor command to ESP32
        Format: "M,left_speed,right_speed\n"
        """
        try:
            with self.lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    return False
                    
                command = f"{self.MSG_MOTOR},{left_speed:.2f},{right_speed:.2f}\n"
                self.serial_conn.write(command.encode())
                return True
                
        except serial.SerialException as e:
            self.logger.error(f"Error sending motor command: {str(e)}")
            return False

    def _read_loop(self):
        """Background thread for reading serial data"""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting:
                    with self.lock:
                        line = self.serial_conn.readline().decode().strip()
                    
                    if line:
                        self._process_message(line)
                        
            except serial.SerialException as e:
                self.logger.error(f"Serial read error: {str(e)}")
                time.sleep(1.0)
                
            except Exception as e:
                self.logger.error(f"Error in read loop: {str(e)}")
                time.sleep(1.0)

    def _process_message(self, message: str):
        """Process incoming messages based on type"""
        try:
            if not message or ',' not in message:
                return
                
            msg_type, *data = message.split(',')
            
            if msg_type == self.MSG_ENCODER and len(data) == 2:
                # Handle encoder data
                left_ticks = int(data[0])
                right_ticks = int(data[1])
                if self.encoder_callback:
                    self.encoder_callback(left_ticks, right_ticks)
                    
            elif msg_type == self.MSG_STATUS:
                # Handle status message
                if self.status_callback:
                    self.status_callback(','.join(data))
                    
            elif msg_type == self.MSG_ERROR:
                # Handle error message
                if self.error_callback:
                    self.error_callback(','.join(data))
                    
        except ValueError as e:
            self.logger.error(f"Error parsing message '{message}': {str(e)}")
            
        except Exception as e:
            self.logger.error(f"Error processing message '{message}': {str(e)}")

    def register_encoder_callback(self, callback: Callable[[int, int], None]):
        """Register callback for encoder data"""
        self.encoder_callback = callback

    def register_status_callback(self, callback: Callable[[str], None]):
        """Register callback for status messages"""
        self.status_callback = callback

    def register_error_callback(self, callback: Callable[[str], None]):
        """Register callback for error messages"""
        self.error_callback = callback

    def flush_input(self):
        """Clear input buffer"""
        if self.serial_conn and self.serial_conn.is_open:
            with self.lock:
                self.serial_conn.reset_input_buffer()

    def flush_output(self):
        """Clear output buffer"""
        if self.serial_conn and self.serial_conn.is_open:
            with self.lock:
                self.serial_conn.reset_output_buffer()

    def send_reset(self) -> bool:
        """Send reset command to ESP32"""
        try:
            with self.lock:
                if not self.serial_conn or not self.serial_conn.is_open:
                    return False
                    
                command = "R\n"  # Reset command
                self.serial_conn.write(command.encode())
                return True
                
        except serial.SerialException as e:
            self.logger.error(f"Error sending reset command: {str(e)}")
            return False