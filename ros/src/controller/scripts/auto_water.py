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
from object_detect.srv import *
from pot_database.srv import *
from controller.srv import *

from datetime import datetime

STOP = 0
WAIT = 1
AUTO_WATER = 11

def check_water_threshold(last_day_str):
    if last_day_str == None:
        return True
    date_format = "%Y-%m-%d %H:%M:%S"
    last_date = datetime.strptime(last_day_str, date_format).date()
    today_date = datetime.now().strftime(date_format).date()
    diff = today_date - last_date
    rospy.loginfo("the date diff is %d..............", diff)
    if diff >= 2:
        return True
    return False

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
        self.server = SimpleActionServer('/ctrl/auto_water/auto_water', AutoWaterAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client 
        self.nav_client = SimpleActionClient('/navigation/navigate', NavigateAction)
        self.nav_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        self.aim_client = SimpleActionClient('/aim', AimAction)
        self.aim_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        # Service Client
        self.all_pots_service = rospy.ServiceProxy('/database/pot/list', GetPotList)
        
        # all pots
        self.pots = self.get_all_pots()

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
        if self.state == AUTO_WATER:
            # 设置任务结果为'cancel'
            result = AutoWaterResult()
            result.result = 'cancel'
            self.server.set_aborted(result)
            self.state = STOP
            rospy.loginfo("Auto Water action server stopped.")
            return StopResponse(True)
        elif self.state == WAIT:
            if self.server.is_active():
                self.server.set_preempted()
            self.state = STOP
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

    def check_flowerpot(self, pot_id):
        """Check the presence of a flowerpot using the object_detect/check_pot service."""
        try:
            check_pot = CheckPotRequest(id=pot_id)
            check_pot_service = rospy.ServiceProxy('/object_detect/check_pot',CheckPot)
            response = check_pot_service(check_pot)

            return response.success
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False
        
    def execute_cb(self,goal):
        self.state = AUTO_WATER

        feedback = AutoWaterFeedback()
        result = AutoWaterResult()
        
        rospy.loginfo("Starting auto watering process")
        
        for i, target in enumerate(self.pots):
            #* 调用导航模块
            if target.active:
                target_pose = target.robot_pose
                success = self.navigate_to_target(target_pose)
                if not success:
                    result.result = 'fail'
                    self.server.set_aborted(result)
                    self.state = WAIT
                    return
                
                if check_water_threshold(target.last_water_date):
                    #* 调用花盆识别模块
                    success = self.check_flowerpot(target.id)
                    if not success:
                        rospy.logwarn("No flowerpot detected")
                        result.result = 'fail'
                        self.server.set_aborted(result)
                        self.state = WAIT
                        return

                    #* 调用浇水模块
                    goal = AimGoal()
                    goal.id = target.id
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
            feedback.target = str(target + 1)
            self.server.publish_feedback(feedback)
            rospy.loginfo("Auto Watering at target %d" % (target + 1))

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

