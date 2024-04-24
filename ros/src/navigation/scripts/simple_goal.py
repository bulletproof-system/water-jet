#!/usr/bin/env python  
# coding=utf-8  
import rospy  
import actionlib  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String

from navigation.msg import *

# 设置速度阈值  
LINEAR_THRESHOLD = 0.00001  # 例如，0.1 m/s  
ANGULAR_THRESHOLD = 0.00001  



class SimpleGoalServer:
    def __init__(self):
        # simple_goal是move_base的client
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
        self.cur_goal = None 
        # 重中之重！！！！reach_flag!!!
        self.reach_flag = True
        self.fail = False
        self.start_move = False
        self.slow_cnt = 0 # 低速计数器
        # feedback和result
        self.feedback = NavigateFeedback()
        self.result = NavigateResult()
        # simple_goal是三个服务层节点的server
        self.server = actionlib.SimpleActionServer("navigate",NavigateAction,self.cb,False)
        self.server.start()
        # 监听速度
        self.vel_sub = None
        rospy.loginfo("导航启动")

    def clear_state(self):
        self.cur_goal = None
        self.reach_flag = True
        self.slow_cnt = 0
        self.fail = False
        self.start_move = False
        
    # server的回调函数
    def cb(self,goal):
        # simple_goal作为move_base的client的回调函数
        def done_cb(state,result):
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.reach_flag = True
            else :
                self.reach_flag = True
                self.fail = True

        def active_cb():
            rospy.loginfo("move_base服务被激活....")


        def fb_cb(fb):
            self.feedback.cur_state = "normal"
            self.server.publish_feedback(self.feedback)   

        # 把goal传递给move_base
        self.cur_goal = MoveBaseGoal()  
        self.cur_goal.target_pose.header.frame_id = "map"  
        self.cur_goal.target_pose.pose.position.x = goal.pos.position.x
        self.cur_goal.target_pose.pose.position.y = goal.pos.position.y
        self.cur_goal.target_pose.pose.position.z = 0 
        self.cur_goal.target_pose.pose.orientation.w = 1.0  
        self.ac.send_goal(self.cur_goal,done_cb,active_cb,fb_cb)  
        self.reach_flag = False
        self.fail = False
        self.slow_cnt = 0
        self.ac.wait_for_server()         
        rospy.loginfo("开始导航……")    
        # 开启监听速度
        self.vel_sub = rospy.Subscriber('/cmd_vel',Twist, self.slow_detect)  

        # 开始监听cancel消息和move_base action的result
        r = rospy.Rate(1)  
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested(): # cancel了
                self.ac.cancel_goal()
                self.result.result = "cancel"
                self.server.set_succeeded(self.result)
                break
            if self.fail == True: # move_base导航失败
                self.result.result = "fail"
                self.server.set_succeeded(self.result)
            if self.reach_flag == True: # 导航成功并到达
                self.result.result = "success"
                self.server.set_succeeded(self.result)
                break
            r.sleep()
        self.clear_state()
        # 停止监听
        self.vel_sub.unregister()
        rospy.loginfo("导航结束！")

    # 监听速度，实现动态避障
    def slow_detect(self,msg): 
        def done_cb(state,result):
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.reach_flag = True
            else :
                self.reach_flag = True
                self.fail = True

        def active_cb():
            rospy.loginfo("move_base服务被激活....")


        def fb_cb(fb):
            self.feedback.cur_state = "normal"
            self.server.publish_feedback(self.feedback) 


        linear = pow(msg.linear.x,2) + pow(msg.linear.y,2)  
        angular = abs(msg.angular.z)
        # 如果速度多次低于阈值，则重新设置航点
        # rospy.loginfo(rospy.loginfo("slow_cnt={}".format(self.slow_cnt)))
        # rospy.loginfo("reach_flag={}".format(self.reach_flag)) 
        if linear > LINEAR_THRESHOLD:
            self.start_move = True 
        if self.slow_cnt == 2 and self.reach_flag == False and self.start_move == True:
            self.slow_cnt = 0
            rospy.logwarn("机器人速度过慢，应该是遇到障碍，正在重新设置航点，新路径计算中...") 
            self.feedback.cur_state = "barrier"
            self.server.publish_feedback(self.feedback) 
            self.ac.cancel_goal()  # 取消当前目标  
            self.ac.send_goal(self.cur_goal,done_cb,active_cb,fb_cb)   
    
        
        if linear  < LINEAR_THRESHOLD and angular < ANGULAR_THRESHOLD and self.reach_flag == False and self.start_move == True:  
            self.slow_cnt += 1
            rospy.sleep(1)
        else :
            self.slow_cnt = 0
    

rospy.init_node("simple_goal")  
server = SimpleGoalServer()
rospy.spin()






