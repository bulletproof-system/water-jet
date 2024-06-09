#!/usr/bin/env python
# coding=utf-8

import rospy
from actionlib import SimpleActionServer,SimpleActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from controller.msg import Hello, NodeInfo
from controller.msg import InspectAction, InspectFeedback, InspectResult
from navigation.msg import NavigateAction, NavigateGoal
from pot_database.srv import *
from object_detect.srv import *
from controller.srv import *


"""
State: 
"""
STOP = 0
WAIT = 1
INSPECTION = 4

class Inspection:
    def __init__(self):
        rospy.init_node('ctrl_inspection')
        
        # state
        self.state = STOP

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
        # Services
        rospy.Service('/ctrl/inspection/start',Start,self.handle_start)
        rospy.Service('/ctrl/inspection/stop',Stop,self.handle_stop)

        # Action Server
        self.server = SimpleActionServer('/ctrl/inspection/inspect', InspectAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client 
        self.nav_client = SimpleActionClient('/navigation/navigate', NavigateAction)
        self.nav_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        # Service Client
        self.pot_info_service = rospy.ServiceProxy('/database/pot/list', GetPotList)
        self.delete_data_service = rospy.ServiceProxy('/database/pot/delete',DeletePot)

    def handle_start(self,req):
        """处理启动服务请求"""
        if self.state == STOP:
            self.state = WAIT  # 将状态设置为 Wait，等待目标任务
            rospy.loginfo("Inspection action server started.")
            return StartResponse(True)
        else:
            rospy.loginfo("Inspection action server already running.")
            return StartResponse(False)    

    def handle_stop(self,req):
        """处理停止服务请求，终止当前任务并重置状态"""
        if self.state == INSPECTION:
            # 设置任务结果为'cancel'
            result = InspectResult()
            result.result = 'cancel'
            self.server.set_aborted(result)
            self.state = STOP
            rospy.loginfo("Inspection action server stopped.")
            return StopResponse(True)
        elif self.state == WAIT:
            if self.server.is_active():
                self.server.set_preempted()
            self.state = STOP
            rospy.loginfo("Inspection action server stopped.")
            return StopResponse(True)
        else:
            rospy.loginfo("Inspection action server is not running.")
            return StopResponse(False)

    def hello_callback(self, msg):
        rospy.loginfo("Received hello message")
        node_info = NodeInfo()
        node_info.mode = INSPECTION
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)

    def get_pose(self):
        try:
            # print("####################################################")
            # print(target_id)
            # print(type(target_id))
            # print("####################################################")
            get_pot_info = GetPotListRequest()
            response = self.pot_info_service(get_pot_info)
            if response.success:
                return response.pots
            else:
                rospy.logwarn("Failed to fetch pots list")
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
            check_pot = CheckPotRequest(id=pot_id)
            check_pot_service = rospy.ServiceProxy('/object_detect/check_pot',CheckPot)
            response = check_pot_service(check_pot)

            return response.success
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
            return False
        
    def execute_cb(self, goal):
        self.state = INSPECTION

        feedback = InspectFeedback()
        result = InspectResult()
        
        pots = get_pot_info()
        
        for i, pot in enumerate(pots):
            feedback.percentage = i * 100 / len(pots)
            feedback.target = pot.id
            self.server.publish_feedback(feedback)

            #* 调用导航模块
            pot_pose = pot.pot_pose
            success = self.navigate_to_target(pot_pose)
            if not success:
                result.result = 'fail'
                rospy.logwarn("inspection: 导航失败")
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 导航到达目的地后，先等待4s，再调用花盆识别
            rospy.sleep(4)

            #* 调用花盆识别模块
            success = self.check_flowerpot(pot.id)
            if not success:
                rospy.logwarn("No flowerpot detected at target: %d" % pot.id)
                rospy.logwarn("开始删花盆,id:%d" % pot.id)
                delete_pot_info = DeletePotRequest(pot.id)
                response = self.delete_data_service(delete_pot_info)
                if response.success:
                    rospy.logwarn("删除成功 id:%d" % pot.id)
                else :
                    rospy.logwarn("删除失败 id:%d" % pot.id)

            feedback.percentage = (i+1) * 100 / len(pots)
            self.server.publish_feedback(feedback)

        rospy.loginfo("Inspection complete")
        result.result = 'success'
        self.server.set_succeeded(result)
        self.state = WAIT

if __name__ == '__main__':

    try:
        inspection = Inspection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
