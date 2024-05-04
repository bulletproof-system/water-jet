#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from controller.msg import Info,Hello
from controller.srv import *

"""
Core State 
uint8 INIT=0
uint8 PENDING=1
uint8 AUTO_MAP=2
uint8 MANUAL_MAP=3
uint8 INSPECTION=4
uint8 TARGET=5
uint8 AUTO_WATER=6
"""

class Core:
    def __init__(self):
        # 初始化节点        
        rospy.init_node("ctrl_core")

        # 当前功能模式
        self.mode = 0

        # 急停模式
        self.scram = False

        # Subscribers
        rospy.Subscriber("/ctrl/cmd_vel",Twist,self._cmd_vel_callback)
        rospy.Subscriber("/hello",Hello,self.hello_callback)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/ctrl/cmd_vel',Twist,queue_size=10)
        self.info_pub = rospy.Publisher('/ctrl/info',Info,queue_size=10)
        
        # Services
        rospy.Service('/ctrl/start',Start,self.handle_start)
        rospy.Service('/ctrl/stop',Stop,self.handle_stop)
        rospy.Service('/ctrl/change_mode',ChangeMode,self.handle_change_mode)
        rospy.Service('/ctrl/scram',Scram,self.scram_callback)

        self.mode = 1
        rospy.wait_for_service('/ctrl/pending/start')
        client = rospy.ServiceProxy('ctrl/pending/start',Start)
        request = StartRequest()
        request.mode = 1
        response = client(request)
    
    def hello_callback(self,hello):
        info = Info(mode=self.mode,scram=self.scram)
        self.info_pub.publish(info)

    def handle_start(self,start):
        mode = start.mode
        if mode != self.mode:
            return StartResponse(success=False)

        service_paths = {
            1: '/ctrl/pending/start',
            2: '/ctrl/auto_map/start',
            3: '/ctrl/manual_map/start',
            4: '/ctrl/inspection/start', 
            5: '/ctrl/target/start',    
            6: '/ctrl/auto_water/start' 
        }

        service_path = service_paths.get(mode)
        rospy.wait_for_service(service_path)

        
        client = rospy.ServiceProxy(service_path, Start)

        try:
            request = StartRequest(mode=mode)
            response = client(request)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
            return StartResponse(success=False)

        return response
    
    def handle_stop(self,stop):
        mode = stop.mode
        if mode != self.mode:
            return StopResponse(success=False)

        service_paths = {
            1: '/ctrl/pending/stop',
            2: '/ctrl/auto_map/stop',
            3: '/ctrl/manual_map/stop',
            4: '/ctrl/inspection/stop', 
            5: '/ctrl/target/stop',    
            6: '/ctrl/auto_water/stop' 
        }
        service_path = service_paths.get(mode)
        rospy.wait_for_service(service_path)

        client = rospy.ServiceProxy(service_path, Stop)
        request = StopRequest(mode=mode)

        response = client(request)
        return response   
         
    def handle_change_mode(self,change_mode):
        mode = change_mode.mode

        service_paths = {
            1: '/ctrl/pending/stop',
            2: '/ctrl/auto_map/stop',
            3: '/ctrl/manual_map/stop',
            4: '/ctrl/inspection/stop', 
            5: '/ctrl/target/stop',    
            6: '/ctrl/auto_water/stop' 
        }

        if mode in service_paths:
            # 关闭当前的mode
            service_path = service_paths[self.mode]
            rospy.wait_for_service(service_path)
            client = rospy.ServiceProxy(service_path, Stop)
            request = StopRequest(mode=self.mode)
            response = client(request)

            if response.success:
                # 如果成功，切换mode
                self.mode = mode
                # 发布info信息
                info = Info(mode=self.mode,scram=self.scram)
                self.info_pub.publish(info)
                change_mode_response = ChangeModeResponse(success=True)
                return change_mode_response
            else:
                change_mode_response = ChangeModeResponse(success=False)
                return change_mode_response
        else:
            # 更换模式出错
            change_mode_response = ChangeModeResponse(success=False)
            return change_mode_response

    def scram_callback(self,scarm):
        self.scram = scarm.active

        # 更新全局状态
        info = Info(mode=self.mode,scram=self.scram)
        self.info_pub.publish(info)

        scarm_response = ScramResponse(success=True)
        return scarm_response

    def _cmd_vel_callback(self,twist):
        # 处于急停状态
        if self.scram:
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.linear.y = 0
            twist_msg.linear.z = 0
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = 0
            self.cmd_vel_pub.publish(twist_msg)
        # 不处于急停状态
        else:
            self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        core = Core()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass