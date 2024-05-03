#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import UInt8MultiArray, String
from actionlib import SimpleActionServer,SimpleActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from controller.msg import Hello, NodeInfo
from controller.msg import TargetAction, TargetFeedback, TargetResult
from navigation.msg import NavigateAction, NavigateGoal
from robot_arm.msg import AimAction,AimGoal
from database.srv import GetPotInfo
from object_detect.srv import CheckPot

class TargetNode:
    def __init__(self):
        rospy.init_node('ctrl_target')
        
        # state
        self.state = 1

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
        # Action Server
        self.server = SimpleActionServer('/ctrl/action/Target', TargetAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client 
        self.nav_client = SimpleActionClient('/navigation/navigate', NavigateAction)
        self.nav_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        self.aim_client = SimpleActionClient('/aim', AimAction)
        self.aim_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        # Service Client
        self.pot_info_service = rospy.ServiceProxy('/database/pot/get', GetPotInfo)

    def hello_callback(self, msg):
        rospy.loginfo("Received hello message")
        node_info = NodeInfo()
        node_info.mode = 5
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)

    def get_target_pose(self, target_id):
        try:
            response = self.pot_info_service(target_id)
            if response.success:
                return response.info.pose
            else:
                rospy.logwarn("Failed to fetch target pose for ID: %s" % target_id)
                return None
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return None        
    
    def navigate_to_target(self, pose):
        goal = NavigateGoal()
        goal.pos = pose
        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result()
        nav_result = self.nav_client.get_result()

        if nav_result.result == 'success':
            return True
        else:
            rospy.logwarn("Navigation failed with result: %s" % nav_result.result)
            return False

    def check_flowerpot(self, pot_id):
        """Check the presence of a flowerpot using the object_detect/check_pot service."""
        try:
            check_pot = CheckPot(id=pot_id)
            check_pot_service = rospy.ServiceProxy('/object_detect/check_pot',CheckPot)
            response = check_pot_service(check_pot)

            return response.success
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False
        
    def execute_cb(self, goal):
        feedback = TargetFeedback()
        result = TargetResult()
        
        rospy.loginfo("Starting watering process for targets: %s" % goal.targets)
        for i, target in enumerate(goal.targets):
            #* 调用导航模块
            target_pose = self.get_target_pose(target)
            success = self.navigate_to_target(target_pose)
            if not success:
                result.result = 'fail'
                self.server.set_aborted(result)
                return

            #* 调用花盆识别模块
            if not self.check_flowerpot(target):
                rospy.logwarn("No flowerpot detected at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                return

            #* 调用浇水模块
            goal = AimGoal()
            goal.id = target
            self.aim_client.send_goal(goal)
            self.aim_client.wait_for_result()
            aim_result = self.aim_client.get_result()

            if not aim_result:
                rospy.logwarn("Unable to aim at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                return
            
            #* 反馈进度
            feedback.percentage = int((i + 1) * 100.0 / len(goal.targets))
            feedback.target = target
            self.server.publish_feedback(feedback)
            rospy.loginfo("Watering at target %d" % target)

            #* target中断
            if self.server.is_preempt_requested():
                rospy.loginfo("Watering preempted")
                self.server.set_preempted()
                result.result = 'cancel'
                return
        
        rospy.loginfo("Watering complete")
        result.result = 'success'
        self.server.set_succeeded(result)

if __name__ == '__main__':

    try:
        target_node = TargetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
