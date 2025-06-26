import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
# from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandInt
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import TwistStamped
import math
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import datetime
from sensor_msgs.msg import Image
import os
from sklearn.cluster import DBSCAN
import numpy as np
import imutils
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from ament_index_python.packages import get_package_share_directory
import threading
import yaml

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        # self.state = State()
        self.position=None
        self.armed = False
        self.offboard_mode = False
        self.arming_future = None
        self.set_mode_future = None
        self.odom=Odometry()
        self.last_req = self.get_clock().now()
        self.width=55.0 #Keep this as exact width dimension of testing ground in m
        self.length=40.0#Keep this as exact length dimension of testing ground in m
        self.ver_fov=0.8517207 #Vertical FOV of camera you are using in radians
        self.hor_fov=1.085595 #Horizontal FOV of camera you are using in radians
        self.frame_width=math.tan(self.hor_fov/2)*30
        self.frame_length=math.tan(self.ver_fov/2)*30
        self.moving_left=True
        self.moving_right=False
        self.prev_detection=False
        self.shapes_prev_detection=False
        self.stop=False
        self.counter=0
        self.hotspots=list()
        self.bridge = CvBridge()
        self.image = None
        self.first_contact=None
        self.shapes_first_contact=None
        self.db_scan_threshold = 40
        self.prev_frame_hotspots=0
        
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create callback groups
        self.cb_1 = MutuallyExclusiveCallbackGroup()
        self.cb_2 = MutuallyExclusiveCallbackGroup()



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
        #self.detection_sub=self.create_subscription(Bool,'hotspot_detections',self.detected_cb,10,callback_group=self.cb_1)
        self.shapes_detection_sub=self.create_subscription(Bool,'shapes_detections',self.shapes_detected_cb,10,callback_group=self.cb_1)
        self.hotspot_sub=self.create_subscription(PoseArray,'/hotspot_posearray',self.hotspot_cb,10,callback_group=self.cb_2)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        self.subscription = self.create_subscription(
            Image,
            '/bottom_camera/image_raw',
            self.image_callback,
            10)
        package_share_directory_yolo = get_package_share_directory('manual_key_control')
        package_share_directory_output = get_package_share_directory('astra_nav_control')
        model_path = model_path = os.path.join(package_share_directory_yolo, 'yolov8', 'shape_counter_3.pt')
        self.output_yaml = '../outputs/outputs.yaml'
        self.image_directory = '../outputs'
        self.model_shapes = YOLO(model_path)
        # Create service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=self.cb_1)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=self.cb_1)
        self.command_int_client = self.create_client(CommandInt, '/mavros/cmd/command_int')

        # Wait for services to be available
        self.wait_for_service(self.arming_client)
        self.wait_for_service(self.set_mode_client)
        self.wait_for_service(self.command_int_client)

        self.set_mav_frame()
        # Publish initial pose setpoints
        for _ in range(50):
            self.publish_pos_setpoint(0.0, 0.0, 2.0)
        self.arm()
    
    def detected_cb(self,msg):
    #     if msg.data==True and self.prev_detection==False:
    #         self.first_contact=self.get_clock().now().nanoseconds / 1e9
    #         print("first contact update")
    #     if self.first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.first_contact > 3.0 and self.stop==False and msg.data==True:
    #         self.hold(5.0)
    #     elif self.first_contact!=None:
    #         print(self.get_clock().now().nanoseconds / 1e9 - self.first_contact)
        self.prev_detection=msg.data

    def hotspot_cb(self,msg:PoseArray):
        if len(msg.poses)>self.prev_frame_hotspots:
            self.first_contact=self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(f"First Contact Hotspot")
        if self.first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.first_contact > 2.0 and len(msg.poses)>0 and self.stop==False:
            self.hold(5.0)

        self.prev_frame_hotspots=len(msg.poses)
        
        # if self.first_contact!=None:
        #     print(self.get_clock().now().nanoseconds / 1e9 - self.first_contact)

    def shapes_detected_cb(self,msg):
        if msg.data==True and self.shapes_prev_detection==False:
            self.shapes_first_contact=self.get_clock().now().nanoseconds / 1e9
            print("first contact update")
        if self.shapes_first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.shapes_first_contact > 3.0 and self.stop==False and msg.data==True:
            self.shapes_hold(5.0)
        elif self.shapes_first_contact!=None:
            print(self.get_clock().now().nanoseconds / 1e9 - self.shapes_first_contact)
        self.shapes_prev_detection=msg.data
        
    def image_callback(self,msg):
        self.image=msg

    def is_far_from_all_hotspots(self):
        for hotspot in self.hotspots:
            distance = self.euclidean_distance([hotspot[0],hotspot[1],0.0], [self.position.x, self.position.y,0.0])
            if distance < 10.0 and self.prev_frame_hotspots<=hotspot[2]:
                return False
        return True
   
    def capture_photo(self):
        """Function to save the latest image with a timestamped filename."""
        try:
            # Convert ROS Image message to OpenCV image and store it
            image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
        if self.image is not None:
            # Generate filename with time and date
            now = datetime.datetime.now()
            filename = now.strftime("%H_%M_%S_%d_%m") + ".jpg"
            current_dir = os.path.dirname(os.path.realpath(__file__))
            save_directory = os.path.join(current_dir, '..', 'hotspots')

            # Ensure the 'hotspots' directory exists
            os.makedirs(save_directory, exist_ok=True)

            # Complete file path with directory and filename
            file_path = os.path.join(save_directory, filename)
            self.hotspots.append([self.position.x, self.position.y,self.prev_frame_hotspots])
            self.stop = False
            try:
                # Save the image
                cv2.imwrite(file_path,image)
                self.get_logger().info(f"Saved image as {filename}")
            except Exception as e:
                self.get_logger().error(f"Failed to save image: {str(e)}")
        else:
            self.get_logger().warn("No image available to save")

    def shapes_capture_photo(self):
        """Function to save the latest image with a timestamped filename."""
        try:
            # Convert ROS Image message to OpenCV image and store it
            image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
            thread = threading.Thread(target= self.count_shapes, args=(image,))
            thread.start()

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")



    def count_shapes(self,image):
    # Load image

        if image is None:
            print("Error: Could not read the image file.")
            return



        # Resize the image to 640x640 for YOLO
        resized_image = cv2.resize(image, (640, 640))

        # Run YOLO model for object detection
        results =self.model_shapes(resized_image)

        # Store boxes by class with confidence scores for clustering
        boxes_by_class = {}
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()  # Bounding box coordinates
            conf = box.conf.item()  # Confidence score
            cls = int(box.cls.item())  # Class index
            class_name =self.model_shapes.names[cls]

            # Filter boxes with low confidence
            if conf > 0.6:
                if cls not in boxes_by_class:
                    boxes_by_class[cls] = []
                boxes_by_class[cls].append(((x1 + x2) / 2, (y1 + y2) / 2))  # Use center points for clustering

        # Cluster boxes by class and count clusters
        final_counts = {}
        for cls, centers in boxes_by_class.items():
            class_name = self.model_shapes.names[cls]

            # Perform clustering on the center points
            clustering = DBSCAN(eps=self.db_scan_threshold, min_samples=1).fit(centers)
            num_clusters = len(set(clustering.labels_))

            # Store the count of unique clusters (representing unique objects)
            final_counts[class_name] = num_clusters

            # Draw bounding boxes and cluster centers on the image
            for cluster_label in set(clustering.labels_):
                cluster_points = [centers[i] for i in range(len(centers)) if clustering.labels_[i] == cluster_label]
                x_center = int(np.mean([pt[0] for pt in cluster_points]))
                y_center = int(np.mean([pt[1] for pt in cluster_points]))

                # Draw cluster center (for visualization)
                cv2.circle(resized_image, (x_center, y_center), 5, (0, 255, 0), -1)
                cv2.putText(resized_image, f"{class_name}", (x_center, y_center - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)

        now = datetime.datetime.now()
        filename = now.strftime("%H_%M_%S_%d_%m") + ".jpg"


        # Ensure the 'hotspots' directory exists
        os.makedirs(self.image_directory, exist_ok=True)

        # Complete file path with directory and filename
        file_path = os.path.join(self.image_directory, filename)
        try:
            # Save the image
            cv2.imwrite(file_path,resized_image)
            self.get_logger().info(f"Saved image as {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {str(e)}")
        print(f"Detection result saved to {file_path}")
        with open(self.output_yaml, 'w') as yaml_file:
            yaml.dump(final_counts, yaml_file)
        print(f"Class counts saved to {self.output_yaml}")


   
    def hold(self,stopping_time):
        if self.is_far_from_all_hotspots():
            self.stop=True
            x=self.position.x
            y=self.position.y
            start_time = self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 - start_time < stopping_time:
                self.publish_pos_setpoint(x, y, 15.0)
                # Log message for feedback
                self.get_logger().info("Stopping, maintaining setpoint...")

            # After 5 seconds, reset the stop flag and counter
            
            self.capture_photo()
        # self.counter = 0
        else:
            self.get_logger().info("Skipping Multiple photos")
        self.first_contact=None
        

    def shapes_hold(self,stopping_time):
        if self.is_far_from_all_hotspots():
            self.stop=True
            x=self.position.x
            y=self.position.y
            start_time = self.get_clock().now().nanoseconds / 1e9
            while self.get_clock().now().nanoseconds / 1e9 - start_time < stopping_time:
                self.publish_pos_setpoint(x, y, 15.0)
                # Log message for feedback
                self.get_logger().info("Stopping, maintaining setpoint...")

            # After 5 seconds, reset the stop flag and counter
            self.stop = False
            
            # self.hotspots.append([self.position.x, self.position.y,0.0])
            self.shapes_capture_photo()
        # self.counter = 0
        self.shapes_first_contact=None
        


        
    # def state_cb(self, msg):
    #     self.state = msg

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
        offb_set_mode.custom_mode = 'OFFBOARD'
        future = self.set_mode_client.call_async(offb_set_mode)
        future.add_done_callback(self.brute_force)
    def brute_force(self,future):
        self.get_logger().info("Offboard Control Mode Activated")
        while self.euclidean_distance([0.0, 0.0, 15.0],[self.position.x,self.position.y,self.position.z])>0.5:
            self.publish_pos_setpoint(0.0, 0.0, 15.0)
        x=self.position.x
        y=self.position.y
        while True:
            if self.stop==True:
                self.get_logger().info("Stopping, maintaining setpoint...")
                # pass
                
            else:
                # self.get_logger().info(f"{self.counter}")
                if self.moving_left==True and self.position.y<self.width/2.0-self.frame_width/2.0:
                    y=self.width/2.0-self.frame_width/2.0
                    self.publish_pos_setpoint(self.position.x,y, 15.0)
                elif self.moving_right!=True:
                    self.moving_left=False
                    x_start=self.position.x
                    x=self.position.x
                    while self.position.x<x_start+self.frame_length/2:
                        x=x_start+self.frame_length/2
                        self.publish_pos_setpoint(x,self.position.y, 15.0)
                    self.moving_right=True
                    
                if self.moving_right==True and self.position.y>-(self.width/2.0-self.frame_width/2.0):
                    self.publish_pos_setpoint(self.position.x,y, 15.0)
                    y=-(self.width/2.0-self.frame_width/2.0)
                elif self.moving_left!=True:
                    self.moving_right=False
                    x_start=self.position.x
                    x=self.position.x
                    while self.position.x<x_start+self.frame_length/2:
                        x=x_start+self.frame_length/2
                        self.publish_pos_setpoint(x,self.position.y, 15.0)
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

    def set_mav_frame(self):
        client = self.create_client(SetParameters, "/mavros/setpoint_velocity/set_parameters")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_parameters service...')


        request = SetParameters.Request()
        param = Parameter()
        param.name = 'mav_frame'
        param.value.type = 4  # Type integer
        param.value.string_value = "BODY_NED"  # Set to FRAME_BODY_NED (1)
        request.parameters = [param]

        # asynchronous call to the service
        future = client.call_async(request)
        future.add_done_callback(self.frame_callback)


    def frame_callback(self,future):
        
        try:

            if future.result().results[0].successful:
                self.get_logger().info("MAV frame set to BODY_NED successfully")
            else:
                self.get_logger().error(f"Failed to set MAV frame. Reason: {future.result().results[0].reason}")
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")


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


# import rclpy
# from std_msgs.msg import Bool
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, PoseArray
# # from mavros_msgs.msg import State
# from mavros_msgs.srv import CommandBool, SetMode, CommandInt
# from nav_msgs.msg import Odometry
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from geometry_msgs.msg import TwistStamped
# import math
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import datetime
# from sensor_msgs.msg import Image
# import os
# from sklearn.cluster import DBSCAN
# import numpy as np
# import imutils
# from rcl_interfaces.srv import SetParameters
# from rcl_interfaces.msg import Parameter
# from ament_index_python.packages import get_package_share_directory
# import threading
# import yaml

# class OffboardNode(Node):
#     def __init__(self):
#         super().__init__('offb_node_py')

#         # self.state = State()
#         self.position=None
#         self.armed = False
#         self.offboard_mode = False
#         self.arming_future = None
#         self.set_mode_future = None
#         self.odom=Odometry()
#         self.last_req = self.get_clock().now()
#         self.width=35.0 #Keep this as exact width dimension of testing ground in m
#         self.length=50.0 #Keep this as exact length dimension of testing ground in m
#         self.ver_fov=0.8517207 #Vertical FOV of camera you are using in radians
#         self.hor_fov=1.085595 #Horizontal FOV of camera you are using in radians
#         self.frame_width=math.tan(self.hor_fov/2)*30
#         self.frame_length=math.tan(self.ver_fov/2)*30
#         self.moving_left=True
#         self.moving_right=False
#         self.prev_detection=False
#         self.shapes_prev_detection=False
#         self.stop=False
#         self.counter=0
#         self.hotspots=list()
#         self.bridge = CvBridge()
#         self.image = None
#         self.first_contact=None
#         self.shapes_first_contact=None
#         self.db_scan_threshold = 40
#         self.prev_frame_hotspots=0
#         self.takeoff=False
        
        
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=1
#         )

#         # Create callback groups
#         self.cb_1 = MutuallyExclusiveCallbackGroup()
#         self.cb_2 = MutuallyExclusiveCallbackGroup()



#         self.local_pos_pub = self.create_publisher(
#             PoseStamped,
#             'mavros/setpoint_position/local',
#             10
#         )

#         self.odom_sub = self.create_subscription(
#             Odometry,
#             'mavros/local_position/odom',
#             self.odom_cb,
#             qos_profile
#         )
#         #self.detection_sub=self.create_subscription(Bool,'hotspot_detections',self.detected_cb,10,callback_group=self.cb_1)
#         self.shapes_detection_sub=self.create_subscription(Bool,'shapes_detections',self.shapes_detected_cb,10,callback_group=self.cb_1)
#         self.hotspot_sub=self.create_subscription(PoseArray,'/hotspot_posearray',self.hotspot_cb,10,callback_group=self.cb_2)
#         self.cmd_vel_pub = self.create_publisher(
#             TwistStamped,
#             '/mavros/setpoint_velocity/cmd_vel',
#             10
#         )
#         self.subscription = self.create_subscription(
#             Image,
#             '/bottom_camera/image_raw',
#             self.image_callback,
#             10)
#         package_share_directory_yolo = get_package_share_directory('manual_key_control')
#         package_share_directory_output = get_package_share_directory('astra_nav_control')
#         model_path = model_path = os.path.join(package_share_directory_yolo, 'yolov8', 'shape_counter_3.pt')
#         self.output_yaml = '../outputs/outputs.yaml'
#         self.image_directory = '../outputs'
#         self.model_shapes = YOLO(model_path)
#         # Create service clients
#         self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming', callback_group=self.cb_1)
#         self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode', callback_group=self.cb_1)
#         self.command_int_client = self.create_client(CommandInt, '/mavros/cmd/command_int')

#         # Wait for services to be available
#         self.wait_for_service(self.arming_client)
#         self.wait_for_service(self.set_mode_client)
#         self.wait_for_service(self.command_int_client)

#         self.set_mav_frame()
#         # Publish initial pose setpoints
#         for _ in range(50):
#             self.publish_pos_setpoint(0.0, 0.0, 2.0)
#         self.arm()
    
#     def detected_cb(self,msg):
#     #     if msg.data==True and self.prev_detection==False:
#     #         self.first_contact=self.get_clock().now().nanoseconds / 1e9
#     #         print("first contact update")
#     #     if self.first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.first_contact > 3.0 and self.stop==False and msg.data==True:
#     #         self.hold(5.0)
#     #     elif self.first_contact!=None:
#     #         print(self.get_clock().now().nanoseconds / 1e9 - self.first_contact)
#         self.prev_detection=msg.data

#     def hotspot_cb(self,msg:PoseArray):
#         if len(msg.poses)>self.prev_frame_hotspots:
#             self.first_contact=self.get_clock().now().nanoseconds / 1e9
#             self.get_logger().info(f"First Contact Hotspot")
#         if self.first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.first_contact > 2.0 and len(msg.poses)>0 and self.stop==False:
#             self.hold(5.0)

#         self.prev_frame_hotspots=len(msg.poses)
        
#         # if self.first_contact!=None:
#         #     print(self.get_clock().now().nanoseconds / 1e9 - self.first_contact)

#     def shapes_detected_cb(self,msg):
#         if msg.data==True and self.shapes_prev_detection==False:
#             self.shapes_first_contact=self.get_clock().now().nanoseconds / 1e9
#             print("first contact update")
#         if self.shapes_first_contact!=None and self.get_clock().now().nanoseconds / 1e9 - self.shapes_first_contact > 3.0 and self.stop==False and msg.data==True:
#             self.shapes_hold(5.0)
#         elif self.shapes_first_contact!=None:
#             print(self.get_clock().now().nanoseconds / 1e9 - self.shapes_first_contact)
#         self.shapes_prev_detection=msg.data
        
#     def image_callback(self,msg):
#         self.image=msg

#     def is_far_from_all_hotspots(self):
#         for hotspot in self.hotspots:
#             distance = self.euclidean_distance([hotspot[0],hotspot[1],0.0], [self.position.x, self.position.y,0.0])
#             if distance < 10.0 and self.prev_frame_hotspots<=hotspot[2]:
#                 return False
#         return True
   
#     def capture_photo(self):
#         """Function to save the latest image with a timestamped filename."""
#         try:
#             # Convert ROS Image message to OpenCV image and store it
#             image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
#         except Exception as e:
#             self.get_logger().error(f"Failed to convert image: {str(e)}")
#         if self.image is not None:
#             # Generate filename with time and date
#             now = datetime.datetime.now()
#             filename = now.strftime("%H_%M_%S_%d_%m") + ".jpg"
#             current_dir = os.path.dirname(os.path.realpath(__file__))
#             save_directory = os.path.join(current_dir, '..', 'hotspots')

#             # Ensure the 'hotspots' directory exists
#             os.makedirs(save_directory, exist_ok=True)

#             # Complete file path with directory and filename
#             file_path = os.path.join(save_directory, filename)
#             self.hotspots.append([self.position.x, self.position.y,self.prev_frame_hotspots])
#             self.stop = False
#             try:
#                 # Save the image
#                 cv2.imwrite(file_path,image)
#                 self.get_logger().info(f"Saved image as {filename}")
#             except Exception as e:
#                 self.get_logger().error(f"Failed to save image: {str(e)}")
#         else:
#             self.get_logger().warn("No image available to save")

#     def shapes_capture_photo(self):
#         """Function to save the latest image with a timestamped filename."""
#         try:
#             # Convert ROS Image message to OpenCV image and store it
#             image = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='bgr8')
#             thread = threading.Thread(target= self.count_shapes, args=(image,))
#             thread.start()

#         except Exception as e:
#             self.get_logger().error(f"Failed to convert image: {str(e)}")



#     def count_shapes(self,image):
#     # Load image

#         if image is None:
#             print("Error: Could not read the image file.")
#             return



#         # Resize the image to 640x640 for YOLO
#         resized_image = cv2.resize(image, (640, 640))

#         # Run YOLO model for object detection
#         results =self.model_shapes(resized_image)

#         # Store boxes by class with confidence scores for clustering
#         boxes_by_class = {}
#         for box in results[0].boxes:
#             x1, y1, x2, y2 = box.xyxy[0].tolist()  # Bounding box coordinates
#             conf = box.conf.item()  # Confidence score
#             cls = int(box.cls.item())  # Class index
#             class_name =self.model_shapes.names[cls]

#             # Filter boxes with low confidence
#             if conf > 0.6:
#                 if cls not in boxes_by_class:
#                     boxes_by_class[cls] = []
#                 boxes_by_class[cls].append(((x1 + x2) / 2, (y1 + y2) / 2))  # Use center points for clustering

#         # Cluster boxes by class and count clusters
#         final_counts = {}
#         for cls, centers in boxes_by_class.items():
#             class_name = self.model_shapes.names[cls]

#             # Perform clustering on the center points
#             clustering = DBSCAN(eps=self.db_scan_threshold, min_samples=1).fit(centers)
#             num_clusters = len(set(clustering.labels_))

#             # Store the count of unique clusters (representing unique objects)
#             final_counts[class_name] = num_clusters

#             # Draw bounding boxes and cluster centers on the image
#             for cluster_label in set(clustering.labels_):
#                 cluster_points = [centers[i] for i in range(len(centers)) if clustering.labels_[i] == cluster_label]
#                 x_center = int(np.mean([pt[0] for pt in cluster_points]))
#                 y_center = int(np.mean([pt[1] for pt in cluster_points]))

#                 # Draw cluster center (for visualization)
#                 cv2.circle(resized_image, (x_center, y_center), 5, (0, 255, 0), -1)
#                 cv2.putText(resized_image, f"{class_name}", (x_center, y_center - 10), 
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (36, 255, 12), 2)

#         now = datetime.datetime.now()
#         filename = now.strftime("%H_%M_%S_%d_%m") + ".jpg"


#         # Ensure the 'hotspots' directory exists
#         os.makedirs(self.image_directory, exist_ok=True)

#         # Complete file path with directory and filename
#         file_path = os.path.join(self.image_directory, filename)
#         try:
#             # Save the image
#             cv2.imwrite(file_path,resized_image)
#             self.get_logger().info(f"Saved image as {filename}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to save image: {str(e)}")
#         print(f"Detection result saved to {file_path}")
#         with open(self.output_yaml, 'w') as yaml_file:
#             yaml.dump(final_counts, yaml_file)
#         print(f"Class counts saved to {self.output_yaml}")


   
#     def hold(self,stopping_time):
#         if self.is_far_from_all_hotspots():
#             self.stop=True
#             x=self.position.x
#             y=self.position.y
#             start_time = self.get_clock().now().nanoseconds / 1e9
#             while self.get_clock().now().nanoseconds / 1e9 - start_time < stopping_time:
#                 self.publish_pos_setpoint(x, y, 15.0)
#                 # Log message for feedback
#                 self.get_logger().info("Stopping, maintaining setpoint...")

#             # After 5 seconds, reset the stop flag and counter
            
#             self.capture_photo()
#         # self.counter = 0
#         else:
#             self.get_logger().info("Skipping Multiple photos")
#         self.first_contact=None
        

#     def shapes_hold(self,stopping_time):
#         if self.is_far_from_all_hotspots():
#             self.stop=True
#             x=self.position.x
#             y=self.position.y
#             start_time = self.get_clock().now().nanoseconds / 1e9
#             while self.get_clock().now().nanoseconds / 1e9 - start_time < stopping_time:
#                 self.publish_pos_setpoint(x, y, 15.0)
#                 # Log message for feedback
#                 self.get_logger().info("Stopping, maintaining setpoint...")

#             # After 5 seconds, reset the stop flag and counter
#             self.stop = False
            
#             # self.hotspots.append([self.position.x, self.position.y,0.0])
#             self.shapes_capture_photo()
#         # self.counter = 0
#         self.shapes_first_contact=None
        


        
#     # def state_cb(self, msg):
#     #     self.state = msg

#     def publish_pos_setpoint(self, x, y, z):
#         pose = PoseStamped()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = z
#         self.local_pos_pub.publish(pose)
#     def publish_velocity(self, linear_x, linear_y, linear_z):
#         twist = TwistStamped()
#         twist.twist.linear.x = linear_x
#         twist.twist.linear.y = linear_y
#         twist.twist.linear.z = linear_z
#         twist.twist.angular.x = 0.0
#         twist.twist.angular.y = 0.0
#         twist.twist.angular.z = 0.0

#         self.cmd_vel_pub.publish(twist)
#         # self.get_logger().info(f"Published velocity command: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})")

#     def odom_cb(self, msg):
#         # Log the current position and orientation of the drone
#         self.odom=msg
#         self.position = msg.pose.pose.position
#         orientation = msg.pose.pose.orientation

#     def euclidean_distance(self,point1, point2):
#         dist=math.sqrt((point1[0] - point2[0])**2 + 
#                         (point1[1] - point2[1])**2 + 
#                         (point1[2] - point2[2])**2)
#         return dist

#     def wait_for_service(self, client):
#         while not client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')

#     def arm(self):
#         arm_cmd = CommandBool.Request()
#         arm_cmd.value = True
#         self.get_logger().info("Requesting Arm")
#         future = self.arming_client.call_async(arm_cmd)
#         future.add_done_callback(self.set_offboard)
#     def set_offboard(self,future):
#         self.get_logger().info("Armed")
#         for _ in range(50):
#             self.publish_pos_setpoint(0.0, 0.0, 2.0)
#         self.get_logger().info("Requesting Offboard Control")
#         offb_set_mode = SetMode.Request()
#         offb_set_mode.custom_mode = 'OFFBOARD'
#         future = self.set_mode_client.call_async(offb_set_mode)
#         future.add_done_callback(self.brute_force)
#     def brute_force(self,future):
#         self.get_logger().info("Offboard Control Mode Activated")
#         while self.euclidean_distance([0.0, 0.0, 15.0],[self.position.x,self.position.y,self.position.z])>0.5:
#             self.publish_pos_setpoint(0.0, 0.0, 15.0)
        
#         x=self.position.x
#         y=self.position.y
#         while True:
#             if self.stop==True:
#                 self.get_logger().info("Stopping, maintaining setpoint...")
#                 # pass
                
#             else:
#                 # self.get_logger().info(f"{self.counter}")
#                 if self.euclidean_distance([5.0, 0.0, 15.0],[self.position.x,self.position.y,self.position.z])>0.5 and self.takeoff==False:
#                         self.publish_pos_setpoint(5.0, 0.0, 15.0)
#                 else:
#                     self.takeoff=True
#                     if self.moving_left==True and self.position.y<self.width/2.0-self.frame_width/2.0:
#                         y=self.width/2.0-self.frame_width/2.0
#                         self.publish_pos_setpoint(self.position.x,y, 15.0)
#                     elif self.moving_right!=True:
#                         self.moving_left=False
#                         x_start=self.position.x
#                         x=self.position.x
#                         while self.position.x<x_start+self.frame_length/2:
#                             x=x_start+self.frame_length/2
#                             self.publish_pos_setpoint(x,self.position.y, 15.0)
#                         self.moving_right=True
                        
#                     if self.moving_right==True and self.position.y>-(self.width/2.0-self.frame_width/2.0):
#                         self.publish_pos_setpoint(self.position.x,y, 15.0)
#                         y=-(self.width/2.0-self.frame_width/2.0)
#                     elif self.moving_left!=True:
#                         self.moving_right=False
#                         x_start=self.position.x
#                         x=self.position.x
#                         while self.position.x<x_start+self.frame_length/2:
#                             x=x_start+self.frame_length/2
#                             self.publish_pos_setpoint(x,self.position.y, 15.0)
#                         self.moving_left=True

#                     if self.position.x>self.length-self.frame_length/2.0+1.0:
#                         self.return_to_home()
#                         self.get_logger().info("Mission Complete Returning To Home")
#                         break

        

#     def return_to_home(self):
#         self.get_logger().info("Returning to home at 15.0 meters altitude")
#         cmd_int = CommandInt.Request()
#         cmd_int.command = 20  # MAV_CMD_NAV_RETURN_TO_LAUNCH
#         cmd_int.frame = 0      # MAV_FRAME_GLOBAL_RELATIVE_ALT
#         cmd_int.current = 0
#         cmd_int.autocontinue = 0
#         cmd_int.param1 = 0.0
#         cmd_int.param2 = 0.0
#         cmd_int.param3 = 0.0
#         cmd_int.param4 = 0.0
#         cmd_int.x = 0         # Not used for RTL
#         cmd_int.y = 0         # Not used for RTL
#         cmd_int.z = 0.0      # RTL altitude

#         future = self.command_int_client.call_async(cmd_int)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result().success:
#             self.get_logger().info("Return to home command sent successfully")
#         else:
#             self.get_logger().error("Failed to send return to home command")

#     def set_mav_frame(self):
#         client = self.create_client(SetParameters, "/mavros/setpoint_velocity/set_parameters")
#         while not client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for set_parameters service...')


#         request = SetParameters.Request()
#         param = Parameter()
#         param.name = 'mav_frame'
#         param.value.type = 4  # Type integer
#         param.value.string_value = "BODY_NED"  # Set to FRAME_BODY_NED (1)
#         request.parameters = [param]

#         # asynchronous call to the service
#         future = client.call_async(request)
#         future.add_done_callback(self.frame_callback)


#     def frame_callback(self,future):
        
#         try:

#             if future.result().results[0].successful:
#                 self.get_logger().info("MAV frame set to BODY_NED successfully")
#             else:
#                 self.get_logger().error(f"Failed to set MAV frame. Reason: {future.result().results[0].reason}")
            
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {str(e)}")


# if __name__ == '__main__':
#     rclpy.init()
#     node = OffboardNode()
#     executor = MultiThreadedExecutor()
#     executor.add_node(node)

#     try:
#         node.get_logger().info('Beginning client, shut down with CTRL-C')
#         executor.spin()
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard interrupt, shutting down.')
#         node.return_to_home()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()