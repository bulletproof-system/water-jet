#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import UInt8MultiArray, String
from actionlib import SimpleActionServer, SimpleActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from controller.msg import *
from controller.msg import Hello, Info, NodeInfo
from controller.srv import *
from map_provider.msg import *
from map_provider.msg import InitMapAction, InitMapGoal, InitMapFeedback, InitMapResult

"""
NodeInfo State: 
"""
STOP = 0
WAIT = 1
MANUAL_INIT_MAP = 8

"""
Info State:
"""
MANUAL_MAP = 3

class ManualMap:
    def __init__(self):
        rospy.init_node('ctrl_manual_map')
        
        # state
        self.state = STOP

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
        # Services
        rospy.Service('/ctrl/manual_map/start', Start, self.handle_start)
        rospy.Service('/ctrl/manual_map/stop', Stop, self.handle_stop)

        # Action Server
        self.server = SimpleActionServer('/ctrl/manual_map/manual_init_map', InitMapAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client
        self.manual_map_client = SimpleActionClient('/map_provider/manual_init_map', InitMapAction)
        if not self.manual_map_client.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("[ManualMap] Failed to connect to the Action Server within 5 seconds.")
        else:
            rospy.loginfo("[ManualMap] Connected to the Action Server.")

    def handle_start(self, req):
        """
        处理启动服务请求
        """
        if self.state == STOP:
            self.state = WAIT           # 将状态设置为 Wait，等待目标任务
            rospy.loginfo("[ManualMap] Action server started.")
            return StartResponse(True)
        else:
            rospy.loginfo("[ManualMap] Action server already running.")
            return StartResponse(False)    

    def handle_stop(self, req):
        """
        处理停止服务请求，终止当前任务并重置状态
        """
        if self.state == MANUAL_INIT_MAP:
            # 设置任务结果为'cancel'
            result = InitMapResult()
            result.result = 'cancel'
            self.server.set_aborted(result)
            self.state = STOP
            rospy.loginfo("[ManualMap] Action server stopped.")
            return StopResponse(True)
        elif self.state == WAIT:
            if self.server.is_active():
                self.server.set_preempted()
            self.state = STOP
            rospy.loginfo("[ManualMap] Action server stopped.")
            return StopResponse(True)
        else:
            rospy.loginfo("[ManualMap] Action server is not running.")
            return StopResponse(False)

    def hello_callback(self, msg):
        rospy.loginfo("[ManualMap] Received hello message.")
        node_info = NodeInfo()
        node_info.mode = MANUAL_MAP
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)
    
    def execute_cb(self, goal):
        self.state = MANUAL_INIT_MAP

        feedback = InitMapFeedback()
        result = InitMapResult()
        
        rospy.loginfo("[ManualMap] Starting manual_map: %s" % goal)

        # 反馈初始进度(始终为-1)
        feedback.percentage = -1
        self.server.publish_feedback(feedback)

        # Call map_provider
        manual_map_goal = InitMapGoal()
        manual_map_goal.caller = "controller"
        self.manual_map_client.send_goal(manual_map_goal)

        # 0520 - new
        # 在这个 while 循环中等待 cancel 请求
        while not rospy.is_shutdown():
            # rospy.loginfo('[ManualMap] into while')
            if self.server.is_preempt_requested():
                rospy.loginfo('[ManualMap] Preempted Manual Map.')
                self.manual_map_client.cancel_goal()
                break
            rospy.sleep(1)  # 休眠，以避免过度占用CPU

        self.manual_map_client.wait_for_result()
        response = self.manual_map_client.get_result()

        if not hasattr(response, 'result') or (response.result != "success"):
            rospy.logwarn("[ManualMap] Failed to complete manual mapping.")
            result.result = "fail"
            self.server.set_aborted(result)
            self.state = WAIT
            return

        rospy.loginfo("[ManualMap] Successfully completed manual mapping.")
        result.result = 'success'
        self.server.set_succeeded(result)
        self.state = WAIT

if __name__ == '__main__':
    try:
        manual_map = ManualMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
