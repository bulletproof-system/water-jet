#!/usr/bin/env python
# coding=utf-8

# -------- 导入模块 ---------------
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from getfrontier import getfrontier

# 订阅者的回调函数
mapData = OccupancyGrid()

def mapCallBack(data):
    global mapData
    mapData = data

# 节点
def node():
    global mapData
    exploration_goal = PointStamped()
    rospy.init_node('detector', anonymous=False)
    
    # 获取参数
    map_topic = rospy.get_param('~map_topic', '/robot_1/map')
    
    # 订阅地图话题
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    
    # 发布检测到的点和标记
    targetspub = rospy.Publisher('/detected_points', PointStamped, queue_size=10)
    pub = rospy.Publisher('shapes', Marker, queue_size=10)
    
    # 等待接收地图数据
    while mapData.header.seq < 1 or len(mapData.data) < 1:
        pass
    
    rate = rospy.Rate(50)
    points = Marker()

    # 设置标记的帧ID和时间戳
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers"
    points.id = 0

    points.type = Marker.POINTS
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0
    points.scale.x = points.scale.y = 0.3
    points.color.r = 255.0 / 255.0
    points.color.g = 0.0 / 255.0
    points.color.b = 0.0 / 255.0
    points.color.a = 1
    points.lifetime = rospy.Duration()

    # 使用 OpenCV 进行前沿检测
    while not rospy.is_shutdown():
        frontiers = getfrontier(mapData)
        for i in range(len(frontiers)):
            x = frontiers[i]
            exploration_goal.header.frame_id = mapData.header.frame_id
            exploration_goal.header.stamp = rospy.Time(0)
            exploration_goal.point.x = x[0]
            exploration_goal.point.y = x[1]
            exploration_goal.point.z = 0

            # 发布检测到的点
            targetspub.publish(exploration_goal)
            points.points = [exploration_goal.point]
            pub.publish(points)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
