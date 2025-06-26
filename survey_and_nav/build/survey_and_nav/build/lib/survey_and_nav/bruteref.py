import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandInt
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TwistStamped
import math
import time

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        self.state = State()
        self.position=None
        self.armed = False
        self.offboard_mode = False
        self.arming_future = None
        self.set_mode_future = None
        self.odom=Odometry()
        self.last_req = self.get_clock().now()
        self.width=100.0
        self.length=200.0
        self.ver_fov=1.0472
        self.hor_fov=1.309
        self.frame_width=math.tan(self.hor_fov/2)*30
        self.frame_length=math.tan(self.ver_fov/2)*30
        self.moving_left=True
        self.moving_right=False
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create callback groups
        self.cb_1 = MutuallyExclusiveCallbackGroup()
        self.cb_2 = MutuallyExclusiveCallbackGroup()

        # Create subscribers and publishers
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10
        )

        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            'mavros/local_position/odom',
            self.odom_cb,
            qos_profile
        )
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )

        # Create service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=self.cb_1)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=self.cb_1)
        self.command_int_client = self.create_client(CommandInt, '/mavros/cmd/command_int')

        # Wait for services to be available
        self.wait_for_service(self.arming_client)
        self.wait_for_service(self.set_mode_client)
        self.wait_for_service(self.command_int_client)

        # Publish initial pose setpoints
        for _ in range(50):
            self.publish_pos_setpoint(0.0, 0.0, 2.0)

        # Create timer for the main loop
        # self.create_timer(0.01, self.set_offboard, callback_group=self.cb_2)
        self.arm()

    def state_cb(self, msg):
        self.state = msg

    def publish_pos_setpoint(self, x, y, z):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        self.local_pos_pub.publish(pose)
    def publish_velocity(self, linear_x, linear_y, linear_z):
        twist = TwistStamped()
        twist.twist.linear.x = linear_x
        twist.twist.linear.y = linear_y
        twist.twist.linear.z = linear_z
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)
        # self.get_logger().info(f"Published velocity command: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})")

    def odom_cb(self, msg):
        # Log the current position and orientation of the drone
        self.odom=msg
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

    def euclidean_distance(self,point1, point2):
        dist=math.sqrt((point1[0] - point2[0])**2 + 
                        (point1[1] - point2[1])**2 + 
                        (point1[2] - point2[2])**2)
        return dist

    def wait_for_service(self, client):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def arm(self):
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        self.get_logger().info("Requesting Arm")
        future = self.arming_client.call_async(arm_cmd)
        future.add_done_callback(self.set_offboard)
    def set_offboard(self,future):
        self.get_logger().info("Armed")
        for _ in range(50):
            self.publish_pos_setpoint(0.0, 0.0, 2.0)
        self.get_logger().info("Requesting Offboard Control")
        offb_set_mode = SetMode.Request()
        offb_set_mode.custom_mode = 'GUIDED_NOGPS'
        future = self.set_mode_client.call_async(offb_set_mode)
        future.add_done_callback(self.brute_force)
    def brute_force(self,future):
        self.get_logger().info("Offboard Control Mode Activated")
        while self.euclidean_distance([0.0, 0.0, 15.0],[self.position.x,self.position.y,self.position.z])>0.5:
            self.publish_pos_setpoint(0.0, 0.0, 15.0)
        x=self.position.x
        y=self.position.y
        while True:
            if self.moving_left==True and self.position.y<self.width/2.0-self.frame_width/2.0:
                self.publish_pos_setpoint(self.position.x,y, 15.0)
                y+=0.05
                time.sleep(0.03)
            elif self.moving_right!=True:
                self.moving_left=False
                x_start=self.position.x
                x=self.position.x
                while x<x_start+self.frame_length/2:
                    self.publish_pos_setpoint(x,self.position.y, 15.0)
                    x+=0.05
                    time.sleep(0.03)
                self.moving_right=True
                
            if self.moving_right==True and self.position.y>-(self.width/2.0-self.frame_width/2.0):
                self.publish_pos_setpoint(self.position.x,y, 15.0)
                y-=0.05
                time.sleep(0.03)
            elif self.moving_left!=True:
                self.moving_right=False
                x_start=self.position.x
                x=self.position.x
                while x<x_start+self.frame_length/2:
                    self.publish_pos_setpoint(x,self.position.y, 15.0)
                    x+=0.05
                    time.sleep(0.03)
                self.moving_left=True

            if self.position.x>self.length-self.frame_length/2.0+1.0:
                self.return_to_home()
                self.get_logger().info("Mission Complete Returning To Home")
                break

        

    def return_to_home(self):
        self.get_logger().info("Returning to home at 15.0 meters altitude")
        cmd_int = CommandInt.Request()
        cmd_int.command = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
        cmd_int.frame = 0      # MAV_FRAME_GLOBAL_RELATIVE_ALT
        cmd_int.current = 0
        cmd_int.autocontinue = 0
        cmd_int.param1 = 0.0
        cmd_int.param2 = 0.0
        cmd_int.param3 = 0.0
        cmd_int.param4 = 0.0
        cmd_int.x = 0         # Not used for RTL
        cmd_int.y = 0         # Not used for RTL
        cmd_int.z = 0.0      # RTL altitude

        future = self.command_int_client.call_async(cmd_int)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Return to home command sent successfully")
        else:
            self.get_logger().error("Failed to send return to home command")

if __name__ == '__main__':
    rclpy.init()
    node = OffboardNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
        node.return_to_home()
    finally:
        node.destroy_node()
        rclpy.shutdown()