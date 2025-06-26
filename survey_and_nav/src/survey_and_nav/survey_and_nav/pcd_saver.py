import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class ZedPcdSaverOnce(Node):
    def __init__(self):
        super().__init__('zed_pcd_saver_once')
        self.saved = False

        self.declare_parameter('output_filename', 'zed_cloud.pcd')
        self.output_filename = self.get_parameter('output_filename').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/mapping/fused_cloud',
            self.cloud_callback,
            10
        )
        self.get_logger().info("Waiting for one fused cloud on /zed/zed_node/mapping/fused_cloud ...")

    def cloud_callback(self, msg):
        if self.saved:
            return

        # Read point cloud as list of (x, y, z) tuples
        points_raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [ [x, y, z] for x, y, z in points_raw ]
        np_points = np.array(points, dtype=np.float32)

        if not points:
            self.get_logger().warn("Received empty point cloud.")
            return

        # Convert to NumPy array
        np_points = np.array(points, dtype=np.float32)

        # Convert to Open3D PointCloud
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(np_points)

        # Save as PCD
        o3d.io.write_point_cloud(self.output_filename, o3d_cloud)
        self.get_logger().info(f"Saved point cloud to {self.output_filename}")
        self.saved = True
        self.subscription = None  # Unsubscribe after saving



def main(args=None):
    rclpy.init(args=args)
    node = ZedPcdSaverOnce()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
