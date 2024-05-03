#!/usr/bin/env python
# coding=utf-8
import tf
import rospy
import multiprocessing
import message_filters
import sqlite3
import os
import pickle
import pcl
import sensor_msgs.point_cloud2 as pcl2
from pcl_ros import point_cloud2 as pcl2_ros
from geometry_msgs.msg import Pose, Point, Quaternion,PointStamped
from sensor_msgs.msg import PointCloud2
from database.msg import PotInfo
from object_detect.srv import CheckPot,CheckPotResponse
from database.srv import SetPotInfo
from math import sqrt

pots = {}                   # pot的信息字典， id -> (x,y,z,last_scan_time) ;
DISTANCE_THRESHOLD = 0.2    # 花盆检测距离阈值
TIMEOUT_THRESHOLD = 5.0     # 检测花盆超时阈值

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

        # Services
        rospy.Service('object_detect/check_pot',CheckPot,self.handle_check_pot)
        
    def load_pots_from_database(self):
        """从数据库加载所有花盆的信息"""
        self.cursor.execute('SELECT id, x, y, z FROM pots')
        rows = self.cursor.fetchall()
        global pots
        pots = {row[0]: {'x': row[1], 'y': row[2], 'z': row[3]} for row in rows}
    
    def handle_check_pot(self,req):
        """检查花盆id所对应的花盆是否存在"""
        pot_id = req.id
        current_time = rospy.get_time() 
        assert pots.get(pot_id) is not None

        last_scan_time = pots.get(pot_id)
        time_difference = current_time - last_scan_time
        
        response = CheckPotResponse()
        if time_difference > TIMEOUT_THRESHOLD:
            response.sucess = False
        else:
            response.sucess = True
        
        return response

    def handle_update_pots(self,obj_center,obj_pointcloud):
        """获取object_center , obj_pointcloud信息,更新花盆信息数据"""

        self.listener.waitForTransform("/map", obj_center.header.frame_id, obj_center.header.stamp, rospy.Duration(5.0))
        world_point = self.listener.transformPoint("/map", obj_center)

        print("###############################")
        print("/base_footprint:")
        print(obj_center.header.stamp)
        print(obj_center.point.x,obj_center.point.y,obj_center.point.z)
        print("/map:")
        print(world_point.header.stamp)
        print(world_point.point.x,world_point.point.y,world_point.point.z)
        print("###############################")

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
            pots[new_id] = {'x': world_point.point.x, 'y': world_point.point.y, 'z': world_point.point.z,'last_scan_time': current_time}

            # 处理点云数据
            cloud = pcl2_ros.read_points_list(obj_pointcloud, field_names=("x", "y", "z", "rgba"), skip_nans=True)
            pcl_cloud = pcl.PointCloud_PointXYZRGBA()
            pcl_cloud.from_list(cloud)
            serialized_cloud = pickle.dumps(pcl_cloud)

            # TODO picture设置
            pot_info = PotInfo()
            pot_info.id = new_id
            pot_info.pose = Pose(Point(world_point.point.x, world_point.point.y, world_point.point.z), Quaternion(0, 0, 0, 1))
            pot_info.data = serialized_cloud
            pot_info.picture = None
            pot_info.active = True
            pot_info.last_water_date = None

            # 调用数据库服务
            set_pot_info_service = rospy.ServiceProxy('/database/pot/set', SetPotInfo)
            response = set_pot_info_service(pot_info)
        


if __name__ == '__main__':
    try:
        object_detector = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass