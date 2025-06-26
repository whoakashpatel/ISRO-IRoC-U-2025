# incomplete, throws warnings and doesn't work as expected
# will be fixed & patched later to the survey_and_nav_node.py

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import StatusText

class QGCLogger(Node):
    def __init__(self):
        super().__init__('qgc_logger')
        self.status_pub = self.create_publisher(StatusText, '/safe-spot-coordinates', 10)
        self.publish_coords()

    def publish_coords(self):
        status_msg = StatusText()
        
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.text = "QGC Logger is running"
        self.status_pub.publish(status_msg)
        
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.text = "QGC2"
        self.status_pub.publish(status_msg)
        
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.text = "QGC3"
        self.status_pub.publish(status_msg)

        self.get_logger().info("Published status: " + status_msg.text)

def main(args=None):
    rclpy.init(args=args)
    qgc_logger = QGCLogger()
    
    try:
        rclpy.spin(qgc_logger)
    except KeyboardInterrupt:
        pass
    finally:
        qgc_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
