#!/usr/bin/env python
# coding=utf-8

import rospy
from geometry_msgs.msg import PointStamped

def callback(data):
    # 这个函数会在每次接收到消息时调用
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data)
    # 你可以在这里添加处理接收到的点的代码
    print("Received a point: x = {}, y = {}, z = {}".format(data.point.x, data.point.y, data.point.z))

def listener():
    # 初始化节点，节点名叫 'listener'
    rospy.init_node('listener', anonymous=True)

    # 订阅 'object_centers' 主题，消息类型为 PointStamped，当有消息到达时调用 callback 函数
    rospy.Subscriber("object_centers", PointStamped, callback)

    # 保持程序运行直到节点被关闭
    rospy.spin()

if __name__ == '__main__':
    listener()