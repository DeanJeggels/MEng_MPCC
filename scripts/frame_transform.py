#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException


class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')
        
        # Declare parameters
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('timeout', 0.2)
        self.declare_parameter('input_topic', '/scan_cloud')
        self.declare_parameter('output_topic', '/ouster/points_base_link')
        
        # Get parameters
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers and publishers
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            10
        )
        
        self.pub = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        self.get_logger().info(f'Transforming point clouds from {input_topic} to frame: {self.target_frame}')
    
    def callback(self, msg):
        try:
            # Lookup transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.timeout)
            )
            
            # Use the tf2 C++ backend through the buffer
            cloud_out = self.tf_buffer.transform(msg, self.target_frame)
            
            # Publish
            self.pub.publish(cloud_out)
            
        except TransformException as e:
            self.get_logger().warn(
                f'Transform failed: {str(e)}',
                throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
