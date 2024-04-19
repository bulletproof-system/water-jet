#!/usr/bin/env python
# coding=utf-8
import tf
import rospy
import multiprocessing
import message_filters
import sqlite3
import os
import pickle

from geometry_msgs.msg import Pose, Point, Quaternion,PointStamped
from sensor_msgs.msg import PointCloud2, Image
from object_detect.srv import SetActive, SetActiveResponse
import sensor_msgs.point_cloud2 as pcl2
from object_detect.msg import PotList, PotInfo
from math import sqrt


pots = {}               # pot的信息字典， id -> (x,y,z) ; pots[id] = {'x': x1, 'y': y1, 'z': z1}
THRESHOLD = 0.5         # 花盆检测距离阈值

class ObjectDetector:
    def __init__(self):
        """初始化Object Detector,包括数据库初始化、订阅话题初始化、发布话题初始化、服务初始化"""
        rospy.init_node("object_detect")
        rospy.loginfo("object detector started!")        

        # 数据库初始化
        self.conn = sqlite3.connect('pots.db')
        self.cursor = self.conn.cursor()
        self.setup_database()

        # 话题订阅初始化
        self.listener = tf.TransformListener()
        object_centers_sub = message_filters.Subscriber('object_centers', Point)
        obj_pointcloud_sub = message_filters.Subscriber('obj_pointcloud', PointCloud2)
        ts = message_filters.ApproximateTimeSynchronizer([object_centers_sub, obj_pointcloud_sub], 10, 0.1)
        ts.registerCallback(self.update_pots)

        # 发布话题初始化
        self.pot_list_pub = rospy.Publisher('/object_detect/pot_list', PotList, queue_size=10)    

        # 服务初始化
        rospy.Service('/object_detect/set_active', SetActive, self.set_active)                        
        rospy.Service('/object_detect/get_pot_list',GetPotList,self.handle_get_pot_list)

    def setup_database(self):
        """设置数据库表格，如果不存在则创建"""
        self.cursor.execute('''
            CREATE TABLE IF NOT EXISTS pots (
                id INTEGER PRIMARY KEY,
                x REAL,
                y REAL,
                z REAL,
                active BOOLEAN,
                image_path TEXT,
                cloud_points BLOB
            )
        ''')
        self.conn.commit()
    
    def update_pots(self,object_center,obj_pointcloud):
        """获取object_center , obj_pointcloud信息,更新花盆信息数据"""
        self.listener.waitForTransform("/world", "/base_footprint", rospy.Time(0), rospy.Duration(4.0))
        world_point = self.listener.transformPoint("/world", object_center)
        
        # 检查是否与已有的花盆坐标相近
        found_match = False
        for pot_id, pot_info in pots.items():
            dist = sqrt((world_point.point.x - pot_info['x'])**2 + 
                            (world_point.point.y - pot_info['y'])**2 +
                            (world_point.point.z - pot_info['z'])**2)
        
            # 根据欧式距离与阈值比较结果
            if dist < THRESHOLD:
                found_match = True
                break
        
            if not found_match:
                # 检测到新的花盆，添加到pots字典中
                new_id = max(pots.keys()) + 1 if pots else 0
                pots[new_id] = {'x': world_point.point.x, 'y': world_point.point.y, 'z': world_point.point.z}
                
                serialized_cloud = pickle.dumps(obj_pointcloud)

                # 写入数据库
                self.cursor.execute('''
                    INSERT INTO pots (id,x, y, z, active, image_path, cloud_points)
                    VALUES (?,?, ?, ?, ?, ?, ?)
                ''', (new_id,world_point.point.x, world_point.point.y, world_point.point.z, False, '', serialized_cloud))
                self.conn.commit()

    def set_active(self, req):
        """更新数据库中的active字段"""
        self.cursor.execute('''
            UPDATE pots SET active = ? WHERE id = ?
        ''', (req.active, req.id))
        self.conn.commit()

        return SetActiveResponse("pot state setting succeeded!")
        
    def publish_pot_list(self, req):
        """从数据库获取特定花盆的对应信息"""
        self.cursor.execute('SELECT * FROM pots WHERE id = ?',(req.id))
        rows = self.cursor.fetchall()

        pot_list = PotList()
        for row in rows:
            pot_info = PotInfo()
            # 设置id
            pot_info.id = row[0]
            
            # 设置active状态
            pot_info.active = row[4]

            # 设置pose
            pose = Pose()
            pose.position = Point(row[1], row[2], row[3])
            pose.orientation = Quaternion(0, 0, 0, 1)
            pot_info.pose = pose
    
            # 设置点云数据
            if row[6]:
                pot_info.data = pickle.loads(row[6])
            else:
                pot_info.data = PointCloud2()
            
            # 设置picture
            try:
                with open(row[5], 'rb') as img_file:
                    img_data = img_file.read()
                    img = Image()
                    img.data = img_data
                    pot_info.picture = img
            except IOError:
                rospy.logwarn("Image file not found for pot id {}".format(pot_info.id))
                pot_info.picture = Image()

            pot_list.potInfos.append(pot_info)
        
        self.pot_list_pub.publish(pot_list)
        rospy.loginfo("Published pot list.")

if __name__ == '__main__':
    ObjectDetector()
    rospy.spin()