#! /usr/bin/env python3

"""
This node listens for a pointcloud message and looks up/extracts a particular transform at the same
timestamp as the pointcloud. It then publishes the transform as a TransformStamped message.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TransformException

class TfRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_node')
        self.pointcloud_topic = self.declare_parameter('pointcloud_topic', '/lidar/pointcloud').get_parameter_value().string_value
        self.parent_frame = self.declare_parameter('parent_frame', 'odom').get_parameter_value().string_value
        self.child_frame = self.declare_parameter('child_frame', 'base_link').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pointcloud_sub = self.create_subscription(PointCloud2, self.pointcloud_topic, self.pointcloud_callback, 10)
        self.get_logger().info(f'TfRelayNode initialized with pointcloud topic: {self.pointcloud_topic}, parent frame: {self.parent_frame}, child frame: {self.child_frame}')

    def pointcloud_callback(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, msg.header.stamp)
        except TransformException as e:
            self.get_logger().error(f'Transform exception: {e}')
            return
        
        transform_stamped = TransformStamped()
        transform_stamped.header = msg.header
        transform_stamped.header.frame_id = self.parent_frame
        transform_stamped.child_frame_id = self.child_frame
        transform_stamped.transform = transform.transform
        self.tf_broadcaster.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TfRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
