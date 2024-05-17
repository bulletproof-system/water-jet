#!/usr/bin/env python
# coding=utf-8
import tf
import rospy
import message_filters
import pickle
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped, Twist
from sensor_msgs.msg import PointCloud2,Image
from pot_database.msg import PotInfo
from object_detect.srv import *
from pot_database.srv import *
from math import sqrt
from cv_bridge import CvBridge

pots = {}                           # pot的信息字典， id -> (x,y,z,last_scan_time) ;
DISTANCE_THRESHOLD = 1.0            # 花盆检测距离阈值
TIMEOUT_THRESHOLD = 5.0             # 检测花盆超时阈值
LINEAR_EPSILON = 0.05               # 线速度阈值
ANGULAR_EPSILON = 0.02              # 角速度阈值

class ObjectDetector:
    def __init__(self):
        """初始化Object Detector,包括数据库初始化、订阅话题初始化、发布话题初始化、服务初始化"""
        rospy.init_node("object_detect")
        rospy.loginfo("object detector started!")        

        # 数据库初始化
        self.load_pots_from_database()

        # Subscribers
        self.listener = tf.TransformListener()
        obj_centers_sub = message_filters.Subscriber('obj_centers', PointStamped)
        obj_pointcloud_sub = message_filters.Subscriber('obj_pointcloud', PointCloud2)
        
        ts = message_filters.ApproximateTimeSynchronizer([obj_centers_sub, obj_pointcloud_sub], 10, 0.1)
        ts.registerCallback(self.handle_update_pots)

        # Separate subscriber for /cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # Subscriber for Kinect2 camera image
        self.image_sub = rospy.Subscriber('/kinect2/hd/image_color_rect', Image, self.image_callback)

        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.latest_image = []
        self.bridge = CvBridge()

        # Services
        rospy.Service('object_detect/check_pot', CheckPot, self.handle_check_pot)
        
    def load_pots_from_database(self):
        """从数据库加载所有花盆的信息"""
        request = GetPotListRequest()
        rospy.wait_for_service('/database/pot/list')
        client = rospy.ServiceProxy('/database/pot/list', GetPotList)
        response = client(request)

        global pots
        pots = {pot_info.id: {'x': pot_info.pose.position.x, 'y': pot_info.pose.position.y, 'z': pot_info.pose.position.z} 
                        for pot_info in response.pots}
        
    def handle_check_pot(self, req):
        """检查花盆id所对应的花盆是否存在"""
        pot_id = req.id
        current_time = rospy.get_time() 
        assert pots.get(pot_id) is not None

        last_scan_time = pots.get(pot_id)
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

    def image_callback(self, img_msg):
        """缓存最新的图像数据"""
        self.latest_image = img_msg
    
    def handle_update_pots(self, obj_center, obj_pointcloud):
        """获取object_center , obj_pointcloud信息,更新花盆信息数据"""

        # 判断当前速度是否不小于某个eps值
        if self.current_linear_velocity >= LINEAR_EPSILON or self.current_angular_velocity >= ANGULAR_EPSILON:
            return

        self.listener.waitForTransform("/map", obj_center.header.frame_id, obj_center.header.stamp, rospy.Duration(5.0))
        world_point = self.listener.transformPoint("/map", obj_center)
        robot_pose = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))

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
            pointcloud_serialized = pickle.dumps(obj_pointcloud)

            # 处理图像数据
            if self.latest_imag != []:
                image_data = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="passthrough")
                image_serialized = pickle.dumps(image_data)
            else:
                image_serialized = []
            
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
