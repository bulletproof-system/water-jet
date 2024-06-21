#! /usr/bin/env python
# coding=utf-8  
import rospy
import actionlib
from navigation.msg import *

from geometry_msgs.msg import Pose

def done_cb(state,result):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("响应结果:result = {}".format(result.result))
        a = 1

def active_cb():
    rospy.loginfo("服务被激活....")
    a = 1


def fb_cb(fb):
    rospy.loginfo("{}".format(fb.percentage))
    rospy.loginfo("当前进度：{}".format(fb.cur_state))

    

if __name__ == "__main__":
    rospy.init_node("target")
    # 创建 action Client 对象
    client = actionlib.SimpleActionClient("/navigation/navigate",NavigateAction)
    print("op = 0:navigate")
    print("op = 1:cancel")
    print("op = -1:exit")
    

    while not rospy.is_shutdown():
        op = int(input("your op = "))
        if op == 0:
            goal_obj = NavigateGoal()
            goal_obj.pos.position.x = float(input("x = "))
            goal_obj.pos.position.y = float(input("y = "))
            goal_obj.pos.position.z = 0
            goal_obj.pos.orientation.w = 1.0
            client.wait_for_server()
            client.send_goal(goal_obj,done_cb,active_cb,fb_cb)
            continue
        if op == 1:
            client.cancel_goal()
            continue
        if op == -1:
            break
    print("exit successfully")
        





    

