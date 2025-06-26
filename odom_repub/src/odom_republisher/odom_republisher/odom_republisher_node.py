# This node transforms ZED odometry to be used by MAVROS and EKF for VISION POSITION ESTIMATION

# Current Implementaiton directly transforms ZED odometry from zed_odom to zed_map frame
# since for our configuration zed_map frame is in the same orientation as body ENU


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import numpy as np

# Monkey patch np.float for compatibility
if not hasattr(np, 'float'):
    np.float = float

import tf_transformations
from builtin_interfaces.msg import Time


class ZedOdomToVisionPose(Node):
    def __init__(self):
        super().__init__('zed_odom_to_vision_pose')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.odom = None

        self.odom_cb_group = MutuallyExclusiveCallbackGroup()
        self.main_cb_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_cb,
            10,
            callback_group=self.odom_cb_group
        )
        
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10, 
            callback_group=self.main_cb_group
            )

        self.initial_orientation_inv = None  # Will store q_initial⁻¹

        self.create_timer(0.03, self.publisher)  # Publish at 10Hz

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def publisher(self):
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                'zed_map',
                self.odom.header.frame_id,
                rclpy.time.Time()
            )

            # Convert odom pose to transform matrix
            T_pose = tf_transformations.quaternion_matrix([
                self.odom.pose.pose.orientation.x,
                self.odom.pose.pose.orientation.y,
                self.odom.pose.pose.orientation.z,
                self.odom.pose.pose.orientation.w
            ])
            T_pose[0:3, 3] = [
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                self.odom.pose.pose.position.z
            ]

            # Transform from zed_odom to zed_map
            T_tf = tf_transformations.quaternion_matrix([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ])
            T_tf[0:3, 3] = [
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            ]

            # Result in zed_map
            T_result = T_tf @ T_pose
            pos_result = T_result[0:3, 3]
            q_current = tf_transformations.quaternion_from_matrix(T_result)

            # Store initial orientation inverse
            if self.initial_orientation_inv is None:
                self.initial_orientation_inv = tf_transformations.quaternion_inverse(q_current)
                self.get_logger().info('Initial orientation saved.')

            # Make current orientation relative to initial
            q_relative = tf_transformations.quaternion_multiply(
                self.initial_orientation_inv,
                q_current
            )

            # Fill PoseStamped
            out = PoseStamped()
            out.header.stamp = self.odom.header.stamp
            out.header.frame_id = 'zed_map'

            out.pose.position.x = pos_result[0]
            out.pose.position.y = pos_result[1]
            out.pose.position.z = pos_result[2]

            out.pose.orientation.x = q_relative[2]
            out.pose.orientation.y = q_relative[1]
            out.pose.orientation.z = -q_relative[0]
            out.pose.orientation.w = q_relative[3]

            self.vision_pub.publish(out)

        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')


def main():
    rclpy.init()
    node = ZedOdomToVisionPose()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.get_logger().info('Destroying node...')
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# Below implementation uses fixed values (as per current cam orientation) to transform odom to body ENU frame

"""
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf_transformations as tft


class ZEDToMAVROSNoTF(Node):
    def __init__(self):
        super().__init__('zed_to_mavros_notf')

        # Precompute the static rotation: zed_odom → odom
        self.q_static = [-0.5, 0.5, 0.5, 0.5]  # x, y, z, w
        self.R_static = tft.quaternion_matrix(self.q_static)[0:3, 0:3]

        self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        self.get_logger().info('Running without TF. Static transform applied manually.')

    def odom_callback(self, msg: Odometry):
        # Extract position vector from ZED odometry
        pos_zed = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Extract ZED quaternion and convert to rotation matrix
        q_zed = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        R_zed = tft.quaternion_matrix(q_zed)[0:3, 0:3]

        # Apply static rotation
        pos_enu = self.R_static @ pos_zed
        R_enu = self.R_static @ R_zed

        # Convert back to quaternion
        q_enu = tft.quaternion_from_matrix(
            np.pad(R_enu, ((0,1),(0,1)), mode='constant', constant_values=0) + np.eye(4)
        )

        # Build PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'odom'

        pose_msg.pose.position.x = pos_enu[0]
        pose_msg.pose.position.y = pos_enu[1]
        pose_msg.pose.position.z = pos_enu[2]

        pose_msg.pose.orientation.x = q_enu[0]
        pose_msg.pose.orientation.y = q_enu[1]
        pose_msg.pose.orientation.z = q_enu[2]
        pose_msg.pose.orientation.w = q_enu[3]

        self.pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZEDToMAVROSNoTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down node.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
