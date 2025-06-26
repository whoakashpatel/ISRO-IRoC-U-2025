import rclpy
import threading
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Quaternion, PoseArray
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import math


class SurveyNode(Node):
    def __init__(self):
        super().__init__('survey_node')

        # callback groups
        self.pose_cb_group = MutuallyExclusiveCallbackGroup()
        self.main_cb_group = MutuallyExclusiveCallbackGroup()

        # survey params
        self.arena_x = 12.0
        self.arena_y = 9.0
        
        self.start_x = 1.6
        self.start_y = 1.6
        self.lane_width = 3.0
        self.position_tolerance = 0.20
        self.survey_alt = 4.0
        
        self.x_offset = 1.5 # x offset from the arena edge
        self.y_offset = 2.0 # y offset from the arena edge

        self.yaw = math.pi / 2.0
        self.orientation_quat = self.yaw_to_quaternion()

        # state variables
        self.waypoints = self.generate_boustrophedon()
        self.waypoint_index = 0
        self.current_pose = None
        self.current_alt = 0.0
        self.state = 'INIT'
        self.safe_spots = None
        self.safe_spot_index = 0
        self.landing_wait_time = 12
        self.safespot_wait_time = 5
        self.survey_completed = False

        # subscribers & publishers
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=pose_qos,
            callback_group=self.pose_cb_group  # pose is added to a separate callback group
        )

        # self.z_sub = self.create_subscription(
        #     PoseStamped,
        #     '', # get z from optical flow
        #     self.z_callback,
        #     qos_profile=pose_qos,
        #     callback_group=self.pose_cb_group
        # )

        safe_spots_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth = 1
        )

        self.subscription = self.create_subscription(
            PoseArray,
            '/safe_spots',
            self.safe_spots_callback,
            qos_profile=safe_spots_qos
        )

        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10,
            callback_group=self.main_cb_group
        )

        # service clients
        self.arming_cli = self.create_client(CommandBool, '/mavros/cmd/arming', callback_group=self.main_cb_group)
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode', callback_group=self.main_cb_group)
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff', callback_group=self.main_cb_group)

        # wait for services
        self.get_logger().info('..................................')
        self.get_logger().info('waiting for mavros services.......')
        self.arming_cli.wait_for_service()
        self.mode_cli.wait_for_service()
        self.takeoff_cli.wait_for_service()
        self.get_logger().info('all mavros services are available.')
        self.get_logger().info('..................................')

        # timer for fsm callback
        self.timer = self.create_timer(0.1, self.fsm_loop, callback_group=self.main_cb_group)


    def yaw_to_quaternion(self):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)
        return q


    def pose_callback(self, msg):
        if not self.current_pose:
            self.get_logger().info('got pose...')
        self.current_pose = msg
        self.current_alt = msg.pose.position.z # remove if using z from optical flow


    # def z_callback(self, msg):
    #     if not self.current_alt:
    #         self.get_logger().info('got altitude...')
    #     self.current_alt = msg.pose.position.z


    def safe_spots_callback(self, msg):
        if not self.safe_spots:
            self.get_logger().info('got safe-spots...')
        self.safe_spots = msg.poses

        # printing safe-spots received
        self.get_logger().info('.............................................')
        self.get_logger().info(f'received {len(self.safe_spots)} safe-spots [x, y, z] --')
        for spot in self.safe_spots:
            self.get_logger().info(f'[{spot.position.x:.4f}, {spot.position.y:.4f}, {spot.position.z:.4f}]')
        self.get_logger().info('.............................................')


    def generate_boustrophedon(self):
        self.get_logger().info('..................................')
        self.get_logger().info('generating boustrophedon waypoints')
    
        wps = []

        x_min = self.x_offset
        x_max = self.arena_x - self.x_offset
        y_min = self.y_offset
        y_max = self.arena_y - self.y_offset
        
        xs = []
        x = x_min
        
        while x <= x_max:
            xs.append(x)
            x += self.lane_width
        
        direction = 1
        
        for x in xs:
            if direction > 0:
                ys = [y_min, y_max]
            else:
                ys = [y_max, y_min]
            for y in ys:
                pos = PoseStamped()
                pos.header.frame_id = 'map'
                pos.pose.position.x = x
                pos.pose.position.y = y
                pos.pose.position.z = self.survey_alt
                pos.pose.orientation = self.orientation_quat
                # time (stamp) is set later while publishing in survey()
                wps.append(pos)
            direction *= -1

        # printing waypoints generated
        self.get_logger().info('.............................................')
        self.get_logger().info(f'generated {len(wps)} waypoints through boustrophedon pattern [x, y, z] --')
        for wp in wps:
            self.get_logger().info(f'[{wp.pose.position.x:.4f}, {wp.pose.position.y:.4f}, {wp.pose.position.z:.4f}]')
        self.get_logger().info('.............................................')

        # adjusting waypoints relative to the start position
        for wp in wps:
            wp.pose.position.y = wp.pose.position.y - self.start_y
            wp.pose.position.x = wp.pose.position.x - self.start_x

        # printing adjusted waypoints
        self.get_logger().info('.............................................')
        self.get_logger().info(f'adjusted {len(wps)} waypoints relative to start position [x, y, z] --')
        for wp in wps:
            self.get_logger().info(f'[{wp.pose.position.x:.4f}, {wp.pose.position.y:.4f}, {wp.pose.position.z:.4f}]')

        self.get_logger().info('done generating waypoints...')
        self.get_logger().info('.............................................')
        
        return wps


    def fsm_loop(self):
        if not self.current_pose:
            return
        
        self.get_logger().info(f'current state: {self.state}')

        if self.state == 'INIT': # mode setting
            self.get_logger().info('setting mode: GUIDED...')

            if self.set_mode('GUIDED'):
                self.state = 'ARM'

        elif self.state == 'ARM': # arming
            self.get_logger().info('arming vehicle...')

            if self.arm(True):
                self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF': # takeoff
            self.get_logger().info(f'taking off to {self.survey_alt}m...')
            
            if self.takeoff(self.survey_alt):
                self.state = 'MONITOR'

        elif self.state == 'MONITOR': # waits till drone is at target altitude
            self.get_logger().info(f'alt: {self.current_alt:.2f} m')
            
            if self.current_alt >= 0.95 * self.survey_alt:
                self.get_logger().info('reached target altitude!...')
                
                if not self.survey_completed:
                    self.get_logger().info('doing survey...')
                    self.state = 'SURVEY'
                
                elif self.safe_spot_index < len(self.safe_spots):
                    self.get_logger().info('moving to next safe-spot...')
                    self.state = 'NAVIGATE'
                
                else:
                    self.get_logger().info('navigation complete! returning...')
                    self.state = 'RETURN'
            
            else:
                time.sleep(0.5)

        elif self.state == 'SURVEY': # survey
            if self.survey():
                self.survey_completed = True
                self.state = 'HOVER' # HOVER / RETURN

        elif self.state == 'HOVER': # hover at last waypoint
            self.hover()
            
            if self.safe_spots is None:
                self.get_logger().info('waiting for safe-spots to be published...')
                time.sleep(0.5)
            
            elif len(self.safe_spots) > 0:
                self.get_logger().info('safe-spots detected, launching navigation...')
                self.state = 'NAVIGATE'
            
            else: # case when no safe-spots
                self.get_logger().info('no safe-spots detected, returning to home...')
                self.state = 'RETURN'

        elif self.state == 'NAVIGATE': # safe spot navigation
            self.navigate()

        elif self.state == 'RETURN': # rtl
            if self.return_to_home():
                self.state = 'DONE'

        elif self.state == 'DONE': # shutdown
            self.get_logger().info('mission complete, shutting down...')
            self.destroy_node()
            rclpy.shutdown()


    def arm(self, arm: bool) -> bool:
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_cli.call_async(req)

        # can try a success check here?

        self.get_logger().info(f'drone {"armed" if arm else "disarmed"}')

        return True


    def set_mode(self, mode: str) -> bool:
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_cli.call_async(req)

        # can try a success check here? 

        self.get_logger().info(f'mode set to {mode}')

        return True


    def takeoff(self, altitude: float) -> bool:
        req = CommandTOL.Request()
        req.altitude  = altitude
        future = self.takeoff_cli.call_async(req)

        # can try a success check here?

        self.get_logger().info(f'drone took off to {altitude} m')

        return True


    def survey(self) -> bool:
        target = self.waypoints[self.waypoint_index]
        
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.z = self.survey_alt

        self.setpoint_pub.publish(target)
        self.get_logger().info(f'publishing target: {self.waypoint_index + 1}/{len(self.waypoints)}')

        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        
        if math.hypot(dx, dy) < self.position_tolerance:
            self.get_logger().info(f'reached waypoint {self.waypoint_index + 1}/{len(self.waypoints)}')
            
            self.waypoint_index += 1

            if self.waypoint_index >= len(self.waypoints):
                self.get_logger().info('survey complete! hovering...')
                return True
        
        return False


    def hover(self):
        self.get_logger().info('hovering at last waypoint...')

        target = self.waypoints[-1]
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.z = self.survey_alt

        self.setpoint_pub.publish(target)


    def navigate(self):
        spot = self.safe_spots[self.safe_spot_index]
        target = PoseStamped()
        target.pose = spot
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.z = self.survey_alt
        target.pose.orientation = self.orientation_quat

        self.setpoint_pub.publish(target)
        self.get_logger().info(f'publishing target: {self.safe_spot_index + 1}/{len(self.safe_spots)}')

        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        
        if math.hypot(dx, dy) < self.position_tolerance:
            self.get_logger().info(f'reached safe-spot {self.safe_spot_index + 1}/{len(self.safe_spots)}, landing...')

            self.set_mode('LAND')
            time.sleep(self.landing_wait_time) # landing wait time

            self.safe_spot_index += 1 # next safe-spot

            self.get_logger().info('landing complete, taking off again...')
            time.sleep(self.safespot_wait_time)  # wait before takeoff

            self.state = 'INIT' # need to be in guided mode to takeoff    


    def return_to_home(self) -> bool:
        self.get_logger().info('publishing home position...')
        
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = 'map'
        target.pose.position.x = 0.0
        target.pose.position.y = 0.0
        target.pose.position.z = self.survey_alt

        self.setpoint_pub.publish(target)

        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        
        if math.hypot(dx, dy) < self.position_tolerance:
            self.get_logger().info('reached home position, landing...')
            
            self.set_mode('LAND')
            time.sleep(self.landing_wait_time)  # landing wait time
            
            return True
        
        return False


        # rtl wouldn't work without GPS

        # self.get_logger().info('returning to home position...')
        # if self.set_mode('RTL'):
        #     self.get_logger().info('return to home initiated.')
        # else:
        #     self.get_logger().error('failed to set RTL mode!')


def main(args=None):
    rclpy.init(args=args)
    node = SurveyNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        node.get_logger().info('....................')
        node.get_logger().info('starting survey node')
        
        executor.spin()
        
        node.get_logger().info('....................')
    
    except KeyboardInterrupt:
        node.get_logger().info('.........................................')
        node.get_logger().info('keyboard interrupt noticed, executing rtl')
        
        node.return_to_home() # rtl
        
        node.get_logger().info('.........................................')
    
    finally:
        # node shuts down after rtl
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
