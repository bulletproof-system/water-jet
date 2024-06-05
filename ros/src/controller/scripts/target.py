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
from pot_database.srv import *
from object_detect.srv import *
from controller.srv import *
from datetime import datetime


"""
State: 
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
        self.server = SimpleActionServer('/ctrl/target/target', TargetAction, execute_cb=self.execute_cb, auto_start=False)
        self.server.start()

        # Action Client 
        self.nav_client = SimpleActionClient('/navigation/navigate', NavigateAction)
        self.nav_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        self.aim_client = SimpleActionClient('/aim', AimAction)
        self.aim_client.wait_for_server(rospy.Duration(5))  # 等待最多5秒以连接服务器

        # Service Client
        self.pot_info_service = rospy.ServiceProxy('/database/pot/get', GetPotInfo)
        self.update_date_service = rospy.ServiceProxy('/database/pot/set_date', SetDate)

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
        if self.state == TARGET:
            # 设置任务结果为'cancel'
            result = TargetResult()
            result.result = 'cancel'
            self.server.set_aborted(result)
            self.state = STOP
            rospy.loginfo("Target action server stopped.")
            return StopResponse(True)
        elif self.state == WAIT:
            if self.server.is_active():
                self.server.set_preempted()
            self.state = STOP
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
            # print("####################################################")
            # print(target_id)
            # print(type(target_id))
            # print("####################################################")
            get_pot_info = GetPotInfoRequest(id=int(target_id))
            response = self.pot_info_service(get_pot_info)
            if response.success:
                return response.info.robot_pose
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
            check_pot = CheckPotRequest(id=pot_id)
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

        #* 反馈初始进度
        assert len(goal.targets) >= 1
        feedback.percentage = 0
        feedback.target = goal.targets[0]
        self.server.publish_feedback(feedback)
        rospy.loginfo("Watering at target %d" % int(goal.targets[0]))
        
        for i, target in enumerate(goal.targets):
            #* 调用导航模块
            target = int(target)
            target_pose = self.get_target_pose(target)
            success = self.navigate_to_target(target_pose)
            if not success:
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 导航到达目的地后，先等待4s，再调用花盆识别
            rospy.sleep(4)

            #* 调用花盆识别模块
            success = self.check_flowerpot(target)
            if not success:
                rospy.logwarn("No flowerpot detected at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 调用浇水模块
            aim_goal = AimGoal()
            aim_goal.id = target
            self.aim_client.send_goal(aim_goal)
            self.aim_client.wait_for_result()
            response = self.aim_client.get_result()
            success = response.success

            if not success:
                rospy.logwarn("Unable to aim at target: %d" % target)
                result.result = 'fail'
                self.server.set_aborted(result)
                self.state = WAIT
                return

            #* 更新浇水日期信息
            try:
                water_date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                update_date_request = SetDateRequest(id=int(target), water_date=water_date)
                update_response = self.update_date_service(update_date_request)
                if not update_response.success:
                    rospy.logwarn("Failed to update watering date for target: %d" % target)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed while updating watering date: %s" % e)
            
            #* 反馈进度
            if i < len(goal.targets):
                feedback.percentage = int((i + 1) * 100.0 / len(goal.targets))
                feedback.target = str(target + 1)
                self.server.publish_feedback(feedback)
                rospy.loginfo("Watering at target %d" % (target + 1))

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
