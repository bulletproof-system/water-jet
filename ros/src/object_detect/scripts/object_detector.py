#!/usr/bin/env python
# coding=utf-8
import tf
import rospy
import message_filters
import pickle
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, Twist
from sensor_msgs.msg import PointCloud2,Image,CameraInfo
from pot_database.msg import PotInfo
from object_detect.srv import *
from pot_database.srv import *
from math import sqrt
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from yolo_detector.msg import BoundingBoxes

pots = {}                           # pot的信息字典， id -> (x,y,z,last_scan_time) ;
DISTANCE_THRESHOLD = 0.2            # 花盆检测距离阈值
TIMEOUT_THRESHOLD = 4.0             # 检测花盆超时阈值
LINEAR_EPSILON = 0.01               # 线速度阈值
ANGULAR_EPSILON = 0.01              # 角速度阈值

class ObjectDetector:
    def __init__(self):
        """初始化Object Detector,包括数据库初始化、订阅话题初始化、发布话题初始化、服务初始化"""
        rospy.init_node("object_detect")
        rospy.loginfo("object detector started!")        

        # 数据库初始化
        self.load_pots_from_database()

        # Subscribers
        self.tf_listener = tf.TransformListener()

        # Define camera intrinsic parameters placeholders
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        self.image_sub = rospy.Subscriber('/yolo_detector/detection_image', Image, self.image_callback)
        self.position_sub = rospy.Subscriber('/yolo_detector/BoundingBoxes', BoundingBoxes, self.position_callback)
        self.depth_sub = rospy.Subscriber('/kinect2/hd/image_depth_rect', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/kinect2/hd/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize variables
        self.detected_image = None
        self.bounding_boxes = None
        self.depth_image = None
        self.latest_image = []

        # Separate subscriber for /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.bridge = CvBridge()
        self.detected_pots = []

        # Publishers
        self.obj_centers_pub = rospy.Publisher('/obj_centers', PointStamped, queue_size=10)

        # Services
        rospy.Service('object_detect/check_pot', CheckPot, self.handle_check_pot)

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def image_callback(self, msg):
        try:
            self.latest_image = msg
            self.detected_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)

    def position_callback(self, msg):
        self.bounding_boxes = msg.bounding_boxes
        if self.detected_image is not None and self.bounding_boxes is not None and self.depth_image is not None:
            for box in self.bounding_boxes:
                if box.Class == "potted plant":
                    rospy.logwarn('successful detect potted plant')
                    # Get the center of the bounding box
                    center_x = (box.xmin + box.xmax) / 2.0
                    center_y = (box.ymin + box.ymax) / 2.0

                    # Get the depth value at the center of the bounding box
                    depth_value = self.depth_image[int(center_y), int(center_x)]
                    rospy.logwarn('successful get depth')

                    # Convert the depth value from millimeters to meters
                    Z = depth_value / 1000.0

                    # Compute the normalized image coordinates
                    normalized_x = (center_x - self.cx) / self.fx
                    normalized_y = (center_y - self.cy) / self.fy

                    # Compute real-world coordinates
                    X = normalized_x * Z
                    Y = normalized_y * Z

                    # Create PointStamped for the detected object
                    point_camera = PointStamped()
                    point_camera.header.frame_id = "kinect2_rgb_optical_frame"
                    point_camera.header.stamp = rospy.Time.now()
                    point_camera.point.x = X
                    point_camera.point.y = Y
                    point_camera.point.z = Z

                    try:
                        # Transform point from camera frame to map frame
                        self.tf_listener.waitForTransform("/map",point_camera.header.frame_id,point_camera.header.stamp,rospy.Duration(1.5))
                        world_point = self.tf_listener.transformPoint("/map", point_camera)
                        robot_pose = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                        rospy.logwarn('successful transform coord')


                        # Transform point from camera frame to base_link frame
                        base_link_point = self.tf_listener.transformPoint("/base_link", point_camera)
                        # Publish to /obj_centers topic
                        self.obj_centers_pub.publish(base_link_point)

                        self.handle_update_pots(world_point,robot_pose,box)
                        rospy.loginfo("Detected potted plant")
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                        rospy.logerr(e)

    def load_pots_from_database(self):
        """从数据库加载所有花盆的信息"""
        request = GetPotListRequest()
        rospy.wait_for_service('/database/pot/list')
        client = rospy.ServiceProxy('/database/pot/list', GetPotList)
        response = client(request)

        global pots
        pots = {pot_info.id: {'x': pot_info.pot_pose.position.x, 'y': pot_info.pot_pose.position.y, 'z': pot_info.pot_pose.position.z} 
                        for pot_info in response.pots}


    def handle_check_pot(self, req):
        """检查花盆id所对应的花盆是否存在"""
        pot_id = req.id
        current_time = rospy.get_time() 
        assert pots.get(pot_id) is not None
        last_scan_time = pots.get(pot_id).get('last_scan_time')
        if last_scan_time is None:  # 没有last_scan_time
            response = CheckPotResponse()
            response.success = False
        else:
            time_difference = current_time - last_scan_time
            
            response = CheckPotResponse()
            if time_difference > TIMEOUT_THRESHOLD:
                response.success = False
            else:
                response.success = True
        
        return response

    def cmd_vel_callback(self, msg):
        """更新当前的速度状态"""
        self.current_linear_velocity = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.linear.z))
        self.current_angular_velocity = max(abs(msg.angular.x), abs(msg.angular.y), abs(msg.angular.z))
    
    def handle_update_pots(self, world_point,robot_pose,box):
        """获取world_point,更新花盆信息数据"""        

        # 判断机器人位置和世界位置不能过于接近
        x1 = world_point.point.x
        y1 = world_point.point.y
        x2 = robot_pose[0][0]
        y2 =  robot_pose[0][1]
        if sqrt((x1 - x2)**2 + (y1 - y2) ** 2 ) < 0.3:
            return 

        # 判断当前速度是否不小于某个eps值
        if self.current_linear_velocity >= LINEAR_EPSILON or self.current_angular_velocity >= ANGULAR_EPSILON:
            return

        # 检查是否与已有的花盆坐标相近
        found_match = False
        dist = 0
        for pot_id, pot_info in pots.items():
            dist = sqrt((world_point.point.x - pot_info['x'])**2 + 
                            (world_point.point.y - pot_info['y'])**2 +
                            (world_point.point.z - pot_info['z'])**2)

            # 根据欧式距离与阈值比较结果
            if dist < DISTANCE_THRESHOLD:
                # 检测到旧的花盆，更新last_scan_time
                found_match = True
                current_time = rospy.get_time() 
                pots[pot_id]['last_scan_time'] = current_time
        
        if not found_match:
            # 检测到新的花盆，添加到pots字典中
            new_id = max(pots.keys()) + 1 if pots else 0
            current_time = rospy.get_time()
            pots[new_id] = {'x': world_point.point.x, 'y': world_point.point.y, 'z': world_point.point.z, 'last_scan_time': current_time}

            # 处理点云数据
            pointcloud_serialized = []

            # 处理图像数据
            image_serialized = []
            if self.latest_image != []:
                image_data = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="passthrough")
                image_serialized = np.array(cv2.imencode(".jpeg",image_data )[1]).tobytes() 

            # 组装PotInfo
            pot_info = PotInfo()
            pot_info.id = new_id
            pot_info.pot_pose = Pose(Point(world_point.point.x, world_point.point.y, world_point.point.z), Quaternion(0, 0, 0, 1))
            pot_info.robot_pose = Pose(Point(robot_pose[0][0], robot_pose[0][1], robot_pose[0][2]), 
            Quaternion(robot_pose[1][0], robot_pose[1][1], robot_pose[1][2], robot_pose[1][3]))            
            pot_info.data = pointcloud_serialized
            pot_info.picture = image_serialized
            pot_info.active = True
            pot_info.last_water_date = ""

            # 调用数据库服务
            set_pot_info_service = rospy.ServiceProxy('/database/pot/set', SetPotInfo)
            request = SetPotInfoRequest()
            request.info = pot_info
            response = set_pot_info_service(request)


if __name__ == '__main__':
    try:
        object_detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
