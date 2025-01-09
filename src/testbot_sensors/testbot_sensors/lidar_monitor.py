#!/usr/bin/env python3
import rclpy
from rclpy.node import Node  # Add this import
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import numpy as np
import time

class LidarMonitor(Node):
    def __init__(self):
        super().__init__('lidar_monitor')
        
        # Initialize variables
        self.total_scans = 0
        self.valid_scans = 0
        self.last_scan_time = time.time()
        
        # Subscribe to scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.monitor_callback,
            10
        )
        
        # Publisher for diagnostics
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Timer for publishing diagnostics
        self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info('LidarMonitor initialized')

    def monitor_callback(self, msg):
        self.total_scans += 1
        valid_readings = [r for r in msg.ranges if not np.isnan(r)]
        if valid_readings:
            self.valid_scans += 1
        self.last_scan_time = time.time()

    def publish_diagnostics(self):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "RPLidar Status"
        status.hardware_id = "rplidar_a1"
        
        # Check lidar health
        time_since_last_scan = time.time() - self.last_scan_time
        if time_since_last_scan > 1.0:
            status.level = DiagnosticStatus.ERROR
            status.message = "No scan data received"
        elif self.valid_scans < self.total_scans * 0.9:
            status.level = DiagnosticStatus.WARN
            status.message = "Low valid scan ratio"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Lidar operating normally"
        
        diag_msg.status.append(status)
        self.diag_pub.publish(diag_msg)

def main(args=None):
    rclpy.init(args=args)
    monitor = LidarMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()