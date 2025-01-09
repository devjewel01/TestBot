#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math

class LidarVisualizer(Node):
    def __init__(self):
        super().__init__('lidar_visualizer')
        
        # Subscribe to scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'lidar_markers',
            10
        )
        
        # Parameters
        self.declare_parameter('max_points', 100)
        self.declare_parameter('point_scale', 0.05)
        
        self.get_logger().info('LidarVisualizer initialized')

    def scan_callback(self, msg):
        marker_array = MarkerArray()
        
        # Point cloud marker
        points_marker = Marker()
        points_marker.header = msg.header
        points_marker.ns = "scan_points"
        points_marker.id = 0
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD
        points_marker.scale.x = self.get_parameter('point_scale').value
        points_marker.scale.y = self.get_parameter('point_scale').value
        points_marker.color.r = 1.0
        points_marker.color.a = 1.0
        
        # Convert scan to points
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or r > msg.range_max or r < msg.range_min:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0
            points_marker.points.append(point)
        
        marker_array.markers.append(points_marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    visualizer = LidarVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()