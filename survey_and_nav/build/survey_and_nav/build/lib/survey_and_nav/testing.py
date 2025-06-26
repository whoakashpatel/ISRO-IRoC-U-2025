#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

qos_profile = QoSProfile(
    reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

class GuidedTakeoffLand(Node):
    def __init__(self):
        super().__init__('testing_node')

        # -- state vars --
        self.current_alt = 0.0
        self.target_alt = 10.0
        self.alt_tolerance = 0.5
        self.state = 'INIT'

        # -- subscribers --
        self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_cb,
            qos_profile=qos_profile
        )

        # -- service clients --
        self.arm_cli     = self.create_client(CommandBool,  '/mavros/cmd/arming')
        self.mode_cli    = self.create_client(SetMode,      '/mavros/set_mode')
        self.takeoff_cli = self.create_client(CommandTOL,    '/mavros/cmd/takeoff')

        self.get_logger().info('Waiting for MAVROS services...')
        self.arm_cli.wait_for_service()
        self.mode_cli.wait_for_service()
        self.takeoff_cli.wait_for_service()
        self.get_logger().info('All MAVROS services available.')

        # drive the sequence at 1 Hz
        self.create_timer(1.0, self.state_machine)

    def pose_cb(self, msg: PoseStamped):
        self.current_alt = msg.pose.position.z

    def state_machine(self):
        if self.state == 'INIT':
            self.get_logger().info('setting mode: GUIDED...')
            if self.set_mode('GUIDED'):
                self.state = 'ARM'
        elif self.state == 'ARM':
            self.get_logger().info('arming vehicle...')
            if self.arm(True):
                self.state = 'TAKEOFF'
        elif self.state == 'TAKEOFF':
            self.get_logger().info(f'taking off to {self.target_alt}m...')
            if self.takeoff(self.target_alt):
                self.state = 'MONITOR'
        elif self.state == 'MONITOR':
            self.get_logger().info(f'alt: {self.current_alt:.2f} m')
            if abs(self.current_alt - self.target_alt) < self.alt_tolerance:
                self.get_logger().info('Target reached → LAND')
                if self.set_mode('LAND'):
                    self.state = 'DONE'
            time.sleep(0.5)
        elif self.state == 'DONE':
            self.get_logger().info('landing initiated, shutting down...')
            self.destroy_node()
            rclpy.shutdown()

    def arm(self, arm: bool) -> bool:
        req = CommandBool.Request()
        req.value = arm
        future = self.arm_cli.call_async(req)

        # can try a success check here?

        return True

    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)

        # can try a success check here?
        # self.get_logger().info(f'see: {future}')

        future.add_done_callback(self.handle_response)

        return True

    def takeoff(self, altitude: float) -> bool:
        req = CommandTOL.Request()
        req.altitude  = altitude
        future = self.takeoff_cli.call_async(req)

        # can try a success check here?

        self.get_logger().info(f'drone took off to {altitude} m')

        return True
    
    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response}')
            print(type(response))
            print(help(response))
        except Exception as e:
            self.get_logger().error(f'Exception during takeoff: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GuidedTakeoffLand()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
