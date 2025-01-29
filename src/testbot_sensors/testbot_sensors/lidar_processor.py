#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LidarProcessor(Node):
    """
    Node for processing RPLidar data and performing additional analysis
    """
    
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('scan_topic', 'scan'),
                ('processed_scan_topic', 'processed_scan'),
                ('laser_frame', 'laser'),
                ('base_frame', 'base_link'),
                ('min_range', 0.5),
                ('max_range', 12.0)
            ]
        )
        
        # Load parameters
        self.scan_topic = self.get_parameter('scan_topic').value
        self.processed_topic = self.get_parameter('processed_scan_topic').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        # Create publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )
        
        self.processed_pub = self.create_publisher(
            LaserScan,
            self.processed_topic,
            10
        )
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Lidar processor initialized')

    def scan_callback(self, msg: LaserScan):
        """
        Process incoming laser scan data
        """
        try:
            # Create copy of scan message for processing
            processed_scan = LaserScan()
            processed_scan.header = msg.header
            processed_scan.angle_min = msg.angle_min
            processed_scan.angle_max = msg.angle_max
            processed_scan.angle_increment = msg.angle_increment
            processed_scan.time_increment = msg.time_increment
            processed_scan.scan_time = msg.scan_time
            processed_scan.range_min = msg.range_min
            processed_scan.range_max = msg.range_max
            
            # Process ranges
            ranges = np.array(msg.ranges)
            intensities = np.array(msg.intensities)
            
            # Apply range filtering
            valid_mask = (ranges >= self.min_range) & (ranges <= self.max_range)
            filtered_ranges = np.where(valid_mask, ranges, float('inf'))
            
            # Update processed scan
            processed_scan.ranges = filtered_ranges.tolist()
            processed_scan.intensities = intensities.tolist()
            
            # Publish processed scan
            self.processed_pub.publish(processed_scan)
            
            # Publish static transform from base to laser
            self.publish_laser_transform(msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing scan: {str(e)}')

    def publish_laser_transform(self, timestamp):
        """
        Publish static transform from base_link to laser frame
        """
        t = TransformStamped()
        
        t.header.stamp = timestamp
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.laser_frame
        
        # Set transform based on robot's physical configuration
        t.transform.translation.x = 0.0  # Adjust based on actual position
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.1  # Height of lidar above base
        
        # No rotation in this case
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LidarProcessor()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error in lidar processor: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()