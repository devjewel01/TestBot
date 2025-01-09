#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import time

class LidarTester(Node):
    def __init__(self):
        super().__init__('lidar_tester')
        
        # Subscribe to scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.test_start_time = time.time()
        self.total_scans = 0
        self.valid_scans = 0
        self.min_range_seen = float('inf')
        self.max_range_seen = 0
        self.scan_frequency = 0
        self.last_scan_time = time.time()
        
        # Create timer for periodic testing status
        self.create_timer(1.0, self.print_test_status)

    def scan_callback(self, msg):
        self.total_scans += 1
        
        # Calculate scan frequency
        current_time = time.time()
        self.scan_frequency = 1.0 / (current_time - self.last_scan_time)
        self.last_scan_time = current_time
        
        # Process valid readings
        valid_readings = [r for r in msg.ranges if not math.isnan(r) and r > msg.range_min and r < msg.range_max]
        if valid_readings:
            self.valid_scans += 1
            self.min_range_seen = min(self.min_range_seen, min(valid_readings))
            self.max_range_seen = max(self.max_range_seen, max(valid_readings))

    def print_test_status(self):
        duration = time.time() - self.test_start_time
        
        self.get_logger().info("\n=== LiDAR Test Status ===")
        self.get_logger().info(f"Test Duration: {duration:.1f} seconds")
        self.get_logger().info(f"Total Scans: {self.total_scans}")
        self.get_logger().info(f"Valid Scans: {self.valid_scans}")
        self.get_logger().info(f"Scan Frequency: {self.scan_frequency:.1f} Hz")
        self.get_logger().info(f"Min Range Seen: {self.min_range_seen:.3f} m")
        self.get_logger().info(f"Max Range Seen: {self.max_range_seen:.3f} m")
        self.get_logger().info("========================\n")

def main(args=None):
    rclpy.init(args=args)
    tester = LidarTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()