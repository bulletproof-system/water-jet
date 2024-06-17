#!/usr/bin/env python
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

def auto_map_send_nav_goal(x, y, z, w, save_map):
    # 创建一个SimpleActionClient，使用move_base的MoveBaseAction接口
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # 创建目标点
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 设置目标位置
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    # 设置目标方向（四元数）
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = w

    rospy.loginfo("[auto_map] First goal (%s, %s, %s, %s) to /move_base", x, y, z, w)
    client.send_goal(goal)

    client.wait_for_result()
    state = client.get_state()

    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("[auto_map] Navigation succeeded!")
        return True
    else:
        rospy.logwarn("[auto_map] Navigation failed with state: %s", state)
        return False

# 示例调用
if __name__ == '__main__':
    rospy.init_node('auto_map_navigation')
    x, y, z, w = 1.0, 2.0, 0.0, 1.0
    save_map = False
    success = auto_map_send_nav_goal(x, y, z, w, save_map)
    if success:
        rospy.loginfo("Robot successfully reached the goal.")
    else:
        rospy.logwarn("Robot failed to reach the goal.")
