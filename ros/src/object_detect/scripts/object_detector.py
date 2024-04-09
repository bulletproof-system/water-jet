#!/usr/bin/env python
# coding=utf-8
import tf
import rospy
import multiprocessing
import message_filters

import os
from geometry_msgs.msg import Pose, Point, Quaternion,PointStamped
from sensor_msgs.msg import PointCloud2, Image
from object_detect.srv import SetActive, SetActiveResponse
import sensor_msgs.point_cloud2 as pcl2
from object_detect.msg import PotList, PotInfo
from math import sqrt


pots = {}         # pot的信息字典， id -> (x,y,z) ; pots[id] = {'x': x1, 'y': y1, 'z': z1}
THRESHOLD = 0.5       # 花盆检测距离阈值

class ObjectDetector:


    def __init__(self):
        rospy.init_node("object_detect")
        rospy.loginfo("object detector started!")        

        # 从DB中读取pots的位置信息
        # 1. TO BE DONE

        self.listener = tf.TransformListener()  # 坐标转换
        object_centers_sub = message_filters.Subscriber('object_centers', Point)
        obj_pointcloud_sub = message_filters.Subscriber('obj_pointcloud', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([object_centers_sub, obj_pointcloud_sub], 10, 0.1)
        ts.registerCallback(self.update_pots)

        rospy.Service('/object_detect/set_active', SetActive, self.set_active)                      # 提供设置是否自动浇灌的服务  
        
        self.pot_list_pub = rospy.Publisher('/object_detect/pot_list', PotList, queue_size=10)      # 发布PotList消息到/object_detect/pot_list话题
        
        rospy.Timer(rospy.Duration(10), self.publish_pot_list)                                      # 每10秒调用一次


    def update_pots(self,object_center,obj_pointcloud): # 获取object_center , obj_pointcloud信息
        self.listener.waitForTransform("/world", "/base_footprint", rospy.Time(0), rospy.Duration(4.0))
        world_point = self.listener.transformPoint("/world", object_center)
        
        # 检查是否与已有的花盆坐标相近
        found_match = False
        for pot_id, pot_info in pots.items():
            dist = sqrt((world_point.point.x - pot_info['x'])**2 + 
                            (world_point.point.y - pot_info['y'])**2 +
                            (world_point.point.z - pot_info['z'])**2)
            if dist < THRESHOLD:
                found_match = True
                break
        
            if not found_match:
                # 检测到新的花盆，添加到pots字典中
                new_id = max(pots.keys()) + 1 if pots else 1
                pots[new_id] = {'x': world_point.point.x, 'y': world_point.point.y, 'z': world_point.point.z}


                # 写入DB
                # 2. TO BE DONE
                # id
                # x,y,z
                # image_path
                # cloud points : from obj_pointcloud


    def set_active(self, req):
        id = req.id             # 目标花盆id
        active = req.active     # 目标状态

        # 将对应DB中花盆表格的active进行设置
        # 3. TO BE DONE

        return SetActiveResponse("pot state setting succeeded!")
        

    def publish_pot_list(self, event):

        pot_list = PotList()

        # 4. TO BE DONE
        # 从数据库获取花盆的对应信息

        pot_info = PotInfo()

        # 设置id active image
        pot_info.id = 1 
        pot_info.active = True 

        # 设置位置 pose

        # 设置点云 data

        pot_list.potInfos.append(pot_info)
        self.pot_list_pub.publish(pot_list)

        rospy.loginfo("Published pot list.")

if __name__ == '__main__':
    ObjectDetector()
    rospy.spin()