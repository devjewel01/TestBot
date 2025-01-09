#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', 'scan'),
                ('min_obstacle_distance', 0.2),  # meters
                ('safety_zone_radius', 0.3),     # meters
                ('scan_timeout', 1.0)            # seconds
            ]
        )
        
        # Get parameters
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.safety_zone_radius = self.get_parameter('safety_zone_radius').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        
        # Initialize variables
        self.last_scan_time = self.get_clock().now()
        self.zones = {
            'front': {'min_angle': -150, 'max_angle': 150},
            'left': {'min_angle': -90, 'max_angle': -30},
            'right':  {'min_angle': 30, 'max_angle': 90},
            'back':  {'min_angle': -30, 'max_angle': 30}
        }
        
        # Subscribe to LaserScan messages
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self.scan_callback,
            10
        )
        
        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer for checking scan timeout
        self.create_timer(0.1, self.check_scan_timeout)
        
        self.get_logger().info('LidarProcessor node initialized')

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.last_scan_time = self.get_clock().now()
        
        # Initialize zone data
        zone_distances = {zone: [] for zone in self.zones.keys()}
        
        # Process each measurement
        for i, distance in enumerate(msg.ranges):
            # Convert array index to angle in degrees
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            
            # Skip invalid measurements
            if math.isnan(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
            
            # Categorize measurement into zones
            for zone, limits in self.zones.items():
                if limits['min_angle'] <= angle <= limits['max_angle']:
                    zone_distances[zone].append(distance)
        
        # Check for obstacles in each zone
        obstacles_detected = False
        for zone, distances in zone_distances.items():
            if distances:  # Only process zones with valid measurements
                min_distance = min(distances)
                if min_distance < self.min_obstacle_distance:
                    self.get_logger().warn(f'Obstacle detected in {zone} zone at {min_distance:.2f}m')
                    obstacles_detected = True
        
        # Publish obstacle detection status
        if obstacles_detected:
            # Send stop command
            stop_cmd = Twist()
            self.obstacle_detected_pub.publish(stop_cmd)

    def check_scan_timeout(self):
        """Check if scan data is being received"""
        current_time = self.get_clock().now()
        if (current_time - self.last_scan_time).nanoseconds / 1e9 > self.scan_timeout:
            self.get_logger().warn('No scan data received for more than {} seconds'.format(
                self.scan_timeout))

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()