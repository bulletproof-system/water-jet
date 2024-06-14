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
AUTO_INIT_MAP = 7

"""
Info State:
"""
AUTO_MAP = 2

class AutoMap:
    def __init__(self):
        rospy.init_node('ctrl_auto_map')
        
        # state
        self.state = STOP

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
        # Services
        rospy.Service('/ctrl/auto_map/start', Start, self.handle_start)
        rospy.Service('/ctrl/auto_map/stop', Stop, self.handle_stop)

        # Action Server
        self.server = SimpleActionServer('/ctrl/auto_map/auto_init_map', InitMapAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client
        self.auto_map_client = SimpleActionClient('/map_provider/auto_init_map', InitMapAction)
        if not self.auto_map_client.wait_for_server(rospy.Duration(5)):
            rospy.logwarn("[AutoMap] Failed to connect to the Action Server within 5 seconds.")
        else:
            rospy.loginfo("[AutoMap] Connected to the Action Server.")

    def handle_start(self, req):
        """
        处理启动服务请求
        """
        if self.state == STOP:
            self.state = WAIT           # 将状态设置为 Wait，等待目标任务
            rospy.loginfo("[AutoMap] Action server started.")
            return StartResponse(True)
        else:
            rospy.loginfo("[AutoMap] Action server already running.")
            return StartResponse(False)    

    def handle_stop(self, req):
        """
        处理停止服务请求，终止当前任务并重置状态
        """
        if self.state == AUTO_INIT_MAP:
            # 设置任务结果为'cancel'
            result = InitMapResult()
            result.result = 'cancel'
            self.server.set_aborted(result)
            self.state = STOP
            rospy.loginfo("[AutoMap] Action server stopped.")
            return StopResponse(True)
        elif self.state == WAIT:
            if self.server.is_active():
                self.server.set_preempted()
            self.state = STOP
            rospy.loginfo("[AutoMap] Action server stopped.")
            return StopResponse(True)
        else:
            rospy.loginfo("[AutoMap] Action server is not running.")
            return StopResponse(False)

    def hello_callback(self, msg):
        rospy.loginfo("[AutoMap] Received hello message.")
        node_info = NodeInfo()
        node_info.mode = AUTO_MAP
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)
    
    def execute_cb(self, goal):
        self.state = AUTO_INIT_MAP

        feedback = InitMapFeedback()
        result = InitMapResult()
        
        rospy.loginfo("[AutoMap] Starting auto_map: %s" % goal)

        # 反馈初始进度(始终为-1)
        feedback.percentage = -1
        self.server.publish_feedback(feedback)

        # Call map_provider
        auto_map_goal = InitMapGoal()
        auto_map_goal.caller = "controller"
        self.auto_map_client.send_goal(auto_map_goal)

        # 在这个 while 循环中等待 cancel 请求
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo('[AutoMap] Preempted Auto Map.')
                self.auto_map_client.cancel_goal()
                break
            if self.auto_map_client.get_result() == None:
                rospy.sleep(1)  # 休眠，以避免过度占用CPU
                continue
            response = self.auto_map_client.get_result()
            if not hasattr(response, 'success') or not response.success:
                rospy.logwarn("[AutoMap] Failed to complete automatic mapping.")
                result.result = "fail"
                self.server.set_aborted(result)
                self.state = WAIT
                return
            rospy.loginfo("[AutoMap] Successfully completed automatic mapping.")
            result.result = 'success'
            self.server.set_succeeded(result)
            self.state = WAIT
            return
        # self.auto_map_client.wait_for_result()
        # response = self.auto_map_client.get_result()
        

if __name__ == '__main__':
    try:
        auto_map = AutoMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
