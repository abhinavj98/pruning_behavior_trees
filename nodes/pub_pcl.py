#THis node publishes a point cloud to the topic /point_cloud

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from sensor_msgs_py import point_cloud2
import open3d as o3d
from std_msgs.msg import Header

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, '/point_cloud', 10)

        # Load PCD file
        self.pcd_file_path = 'tree.pcd'  
        self.frame_id = 'world'  

        # Publish the PointCloud2 message
        self.timer = self.create_timer(1.0, self.publish_pcd)

    def publish_pcd(self):
        try:
            # Load PCD file using Open3D
            pcd = o3d.io.read_point_cloud(self.pcd_file_path)
            points = np.asarray(pcd.points)

            # Create a Header
            header = Header()
            header.stamp = self.get_clock().now().to_msg()  # Proper timestamp
            header.frame_id = self.frame_id  # Proper frame ID

            # Convert points to PointCloud2 format
            point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())

            # Publish the message
            self.publisher_.publish(point_cloud_msg)
            self.get_logger().info('Published PointCloud2 message.')
        except Exception as e:
            self.get_logger().error(f'Failed to publish PointCloud2: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
