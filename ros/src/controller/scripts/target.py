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
from controller.srv import Start,StartResponse,Stop,StopResponse
"""
State: 

uint8 Stop=0
uint8 Wait=1
uint8 Target=10
"""
STOP = 0
WAIT = 1
TARGET = 10
class Target:
    def __init__(self):
        rospy.init_node('ctrl_target')
        
        # state
        self.state = STOP

        # Subscribers
        self.hello_subscriber = rospy.Subscriber('hello', Hello, self.hello_callback)
        
        # Publishers
        self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
        # Services
        rospy.Service('/ctrl/target/start',Start,self.handle_start)
        rospy.Service('/ctrl/target/stop',Stop,self.handle_stop)

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
    
    def handle_start(self,req):
        """处理启动服务请求"""
        if self.state == STOP:
            self.state = WAIT  # 将状态设置为 Wait，等待目标任务
            rospy.loginfo("Target action server started.")
            return StartResponse(True)
        else:
            rospy.loginfo("Target action server already running.")
            return StartResponse(False)    

    def handle_stop(self,req):
        """处理停止服务请求，终止当前任务并重置状态"""
        if self.state != STOP:
            self.state = STOP
            if self.server.is_active():
                self.server.set_preempted()  # 如果有活动的目标任务，提前终止
            rospy.loginfo("Target action server stopped.")
            return StopResponse(True)
        else:
            rospy.loginfo("Target action server is not running.")
            return StopResponse(False)

    def hello_callback(self, msg):
        rospy.loginfo("Received hello message")
        node_info = NodeInfo()
        node_info.mode = 5
        node_info.state = self.state
        self.node_info_publisher.publish(node_info)

    def get_target_pose(self, target_id):
        try:
            get_pot_info = GetPotInfo()
            get_pot_info.id = target_id
            response = self.pot_info_service(get_pot_info)
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
        self.state = TARGET

        feedback = TargetFeedback()
        result = TargetResult()
        
        rospy.loginfo("Starting watering process for targets: %s" % goal.targets)
        for i, target in enumerate(goal.targets):
            #* 调用导航模块
            target_pose = self.get_target_pose(target)
            response = self.navigate_to_target(target_pose)
            if not response.success:
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 调用花盆识别模块
            response = self.check_flowerpot(target)
            if not response.success:
                rospy.logwarn("No flowerpot detected at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 调用浇水模块
            goal = AimGoal()
            goal.id = target
            self.aim_client.send_goal(goal)
            self.aim_client.wait_for_result()
            response = self.aim_client.get_result()

            if not response.success:
                rospy.logwarn("Unable to aim at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
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
                self.state = WAIT
                return
        
        rospy.loginfo("Watering complete")
        result.result = 'success'
        self.server.set_succeeded(result)
        self.state = WAIT

if __name__ == '__main__':

    try:
        target = Target()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
