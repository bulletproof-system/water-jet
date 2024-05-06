#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import UInt8MultiArray, String
from actionlib import SimpleActionServer,SimpleActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from controller.msg import Hello, NodeInfo
from controller.msg import AutoWaterAction, AutoWaterFeedback, AutoWaterResult
from navigation.msg import NavigateAction, NavigateGoal
from robot_arm.msg import AimAction,AimGoal
from pot_database.srv import GetPotList
from controller.srv import Start,StartResponse,Stop,StopResponse

STOP = 0
WAIT = 1
AUTO_WATER = 11

class AutoWaterNode:
    def __init__(self):
        rospy.init_node('ctrl_auto_water')
        
        # state
        self.state = STOP

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)

        # Services
        rospy.Service('/ctrl/auto_water/start',Start, self.handle_start)
        rospy.Service('/ctrl/auto_water/stop',Stop, self.handle_stop)
        
        # Action Server
        self.server = SimpleActionServer('/ctrl/action/AutoWater', AutoWaterAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client 
        self.nav_client = SimpleActionClient('/navigation/navigate', NavigateAction)
        self.nav_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        self.aim_client = SimpleActionClient('/aim', AimAction)
        self.aim_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        # Service Client
        self.all_pots_service = rospy.ServiceProxy('/database/pot/list', GetPotList)
        
        # all pots
        self.pots = get_all_pots()

    def handle_start(self,req):
        """处理启动服务请求"""
        if self.state == STOP:
            self.state = WAIT  # 将状态设置为 Wait，等待目标任务
            rospy.loginfo("Auto Water action server started.")
            return StartResponse(True)
        else:
            rospy.loginfo("Auto Water server already running.")
            return StartResponse(False)    

    def handle_stop(self,req):
        """处理停止服务请求，终止当前任务并重置状态"""
        if self.state != STOP:
            self.state = STOP
            if self.server.is_active():
                self.server.set_preempted()  # 如果有活动的目标任务，提前终止
            rospy.loginfo("Auto Water action server stopped.")
            return StopResponse(True)
        else:
            rospy.loginfo("Auto Water action server is not running.")
            return StopResponse(False)

    def get_all_pots(self):
        response = self.all_pots_service()
        return response.pots
        
    def hello_callback(self, msg):
        rospy.loginfo("Received hello message")
        node_info = NodeInfo()
        node_info.mode = 6
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)     
    
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
        
    def execute_cb(self, goal):
        self.state = AUTO_WATER

        feedback = AutoWaterFeedback()
        result = AutoWaterResult()
        
        rospy.loginfo("Starting auto watering process")
        for i, target in enumerate(self.pots):
            #* 调用导航模块
            target_pose = target.pose
            success = self.navigate_to_target(target_pose)
            if not success:
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 调用浇水模块
            goal = AimGoal()
            goal.id = target
            self.aim_client.send_goal(goal)
            self.aim_client.wait_for_result()
            aim_result = self.aim_client.get_result()

            if not aim_result.success:
                rospy.logwarn("Unable to aim at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return
            
            #* 反馈进度
            feedback.percentage = int((i + 1) * 100.0 / len(self.pots))
            feedback.target = target
            self.server.publish_feedback(feedback)
            rospy.loginfo("Auto Watering at target %d" % target)

            #* 中断
            if self.server.is_preempt_requested():
                rospy.loginfo("Auto Watering preempted")
                self.server.set_preempted()
                result.result = 'cancel'
                self.state = WAIT
                return
        
        rospy.loginfo("Auto Watering complete")
        result.result = 'success'
        self.server.set_succeeded(result)
        self.state = WAIT

if __name__ == '__main__':
    try:
        auto_water_node = AutoWaterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

