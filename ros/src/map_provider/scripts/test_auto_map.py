#!/usr/bin/env python
# coding=utf-8

import rospy
import actionlib
from map_provider.msg import InitMapAction, InitMapGoal

def test_auto_map():
    rospy.init_node('test_auto_map_client')

    # 创建一个SimpleActionClient
    client = actionlib.SimpleActionClient('/map_provider/auto_init_map', InitMapAction)

    rospy.loginfo("Waiting for action server to start...")
    client.wait_for_server()

    rospy.loginfo("Action server started, sending goal.")
    goal = InitMapGoal()
    # 根据需要设置goal的参数
    client.send_goal(goal)

    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Result: %s", result.result)

if __name__ == '__main__':
    try:
        test_auto_map()
    except rospy.ROSInterruptException:
        pass
