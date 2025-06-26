# This node sets the fake initial EKF origin for VIsion Pose Estimation
# This functionality will be further patched to the main survey_and_nav pkg

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPointStamped

class SetEkfOriginNode(Node):
    def __init__(self):
        super().__init__('set_ekf_origin_node')

        # set_gp_origin publisher
        self.publisher_ = self.create_publisher(
            GeoPointStamped,
            '/mavros/global_position/set_gp_origin',
            10
        )

        self.timer = self.create_timer(2.0, self.set_origin)
        self.origin_set = False

    def set_origin(self):
        if self.origin_set:
            return

        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # pick origin coordinates through maps (needn't set correct altitude thus 0.0)
        msg.position.latitude = 12.837691
        msg.position.longitude = 80.138096
        msg.position.altitude = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f"Published EKF origin: {msg.position}")
        self.origin_set = True  # to publish only once

def main(args=None):
    rclpy.init(args=args)
    node = SetEkfOriginNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
