#!/usr/bin/env python
# coding=utf-8

# import rospy
# import actionlib
# from map_provider.msg import *

# def send_goal_to_map_server():
#     # 等待action server启动
#     rospy.loginfo("Waiting for action server to start.")
#     client.wait_for_server()
#     rospy.loginfo("Action server started, sending goal.")

#     # 创建目标(goal)
#     goal = InitMapGoal()
#     # 使用send_goal向server发送请求
#     # client.send_goal(goal, done_cb=done_callback, feedback_cb=feedback_callback)
#     client.cancel_goal()
    
#     rospy.loginfo("Goal sent to action server.")

# def done_callback(status, result):
#     if status == actionlib.GoalStatus.SUCCEEDED:
#         rospy.loginfo("Action completed successfully.")
#     elif status == actionlib.GoalStatus.PREEMPTED:
#         rospy.loginfo("Action was preempted.")
#     else:
#         rospy.loginfo("Action terminated with status: %s", status)

# def feedback_callback(feedback):
#     rospy.loginfo("Current progress: %d%%", feedback.percentage)

# if __name__ == '__main__':
#     try:
#         # 初始化节点
#         rospy.init_node('manual_map_action_client')
#         # 连接到action server
#         client = actionlib.SimpleActionClient('/map_provider/manual_init_map', InitMapAction)
        
#         # 发送目标(goal)到action server
#         send_goal_to_map_server()
        
#         # 等待结果
#         client.wait_for_result()

#         # 输出结果
#         result = client.get_result()
#         rospy.loginfo("Result: %s", result.result)

#     except rospy.ROSInterruptException:
#         rospy.loginfo("Program interrupted before completion.")


import time
import rospy
import actionlib
from map_provider.msg import *

def feedback_cb(feedback):
    rospy.loginfo('Feedback received: Current mapping progress: %d%%', feedback.percentage)

def call_manual_mapping():
    rospy.init_node('test_map_provider_client')
    client = actionlib.SimpleActionClient('/map_provider/manual_init_map', InitMapAction)
    
    # 等待 action server 启动
    rospy.loginfo("Waiting for action server to start.")
    client.wait_for_server()

    # 发送启动建图的目标
    rospy.loginfo("Starting manual mapping...")
    goal = InitMapGoal(caller='test_client')
    client.send_goal(goal, feedback_cb=feedback_cb)
    
    # 在这里等待一些时间，以观察是否有反馈信息返回
    # rospy.sleep(5)  # 等待5秒以便收集反馈信息

    # 等待结果
    # rospy.loginfo("Waiting for the result...")
    # client.wait_for_result()
    # result = client.get_result()
    # rospy.loginfo("Result of mapping: %s", result.result)

    # 发送停止建图的指令
    rospy.loginfo("Cancelling mapping...")
    client.cancel_goal()

    # 再次等待以确认取消是否成功
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    rospy.loginfo("Mapping cancelled.")

    rospy.loginfo("Test completed.")

if __name__ == '__main__':
    try:
        call_manual_mapping()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted before completion.")
