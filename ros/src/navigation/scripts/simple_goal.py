#!/usr/bin/env python  
# coding=utf-8  

import rospy  
import actionlib  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String
from nav_msgs.msg import Path

from navigation.msg import *

# 设置速度阈值  
LINEAR_THRESHOLD = 0.00001  # 例如，0.1 m/s  
ANGULAR_THRESHOLD = 0.00001  


# 状态转移: 
#           planning -> normal
#           normal   -> barrier
#           barrier  -> planning
#           normal   -> reach
#           planning -> fail 
# 重要的状态变量：
#           plan_init   路径规划完成标志
#           slow_cnt    低速计数器
#           reach_flag  导航结束标志 
#           fail        路径规划失败标志      

class SimpleGoalServer:
    def __init__(self):
        self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction) # simple_goal是move_base的client
        self.cur_goal = None 
        self.global_plan = None # 存路径

        # 状态
        self.reach_flag = True
        self.fail = False
        self.slow_cnt = 0 # 低速计数器
        self.cur_pos_id = 0 # 存当前进度
        self.plan_init = False # 全局路径是否规划完成

        # feedback和result
        self.feedback = NavigateFeedback()
        self.result = NavigateResult()

        # simple_goal是三个服务层节点的server
        self.server = actionlib.SimpleActionServer("navigate",NavigateAction,self.simple_goal_cb,False)
        self.server.start()

        # 监听速度
        self.vel_sub = None
        # 监听global_planner
        self.gp_sub = None
        rospy.loginfo("导航启动")

    def clear_state(self):
        self.cur_goal = None
        self.reach_flag = True
        self.slow_cnt = 0
        self.fail = False
        self.global_plan = None 
        self.cur_pos_id = 0 
        self.plan_init = False
        
    # server的回调函数
    def simple_goal_cb(self,goal):
        # simple_goal作为move_base的client的回调函数
        def move_base_done_cb(state,result):
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.reach_flag = True
            else :
                self.reach_flag = True
                self.fail = True

        def updata_global_plan(msg):
            p_list = msg.poses # geometry_msgs/PoseStamped[]
            self.global_plan = []
            self.cur_pos_id = 0
            for p in p_list: # geometry_msgs/PoseStamped
                pose = p.pose # geometry_msgs/Pose
                x = pose.position.x
                y = pose.position.y
                z = pose.orientation.z
                self.global_plan.append((x,y,z))
            if len(self.global_plan) == 0:
                return
            self.plan_init = True

        def move_base_active_cb():
            rospy.loginfo("move_base服务被激活....")


        def move_base_fb_cb(fb):
            if self.plan_init == False:
                self.feedback.cur_state = "planning"
                self.feedback.percentage = 0
                self.server.publish_feedback(self.feedback) 
                return
            self.feedback.cur_state = "normal"
            x = fb.base_position.pose.position.x
            y = fb.base_position.pose.position.y 
            z = fb.base_position.pose.orientation.z
            # 计算进度
            min_dis = None
            tar = None
            for i in range(self.cur_pos_id,len(self.global_plan)):
                dis = pow((x-self.global_plan[i][0]),2) + pow(y-self.global_plan[i][1],2) + 0.1*abs(z-self.global_plan[i][2])
                if tar == None or dis < min_dis:
                    min_dis = dis
                    tar = i
                    
            
            self.cur_pos_id = tar
            self.feedback.percentage = int(100 * float(self.cur_pos_id + 1) / len(self.global_plan))
            self.server.publish_feedback(self.feedback) 
        
        # 初始化状态
        self.clear_state()
        # 把goal传递给move_base
        self.cur_goal = MoveBaseGoal()  
        self.cur_goal.target_pose.header.frame_id = "map"  
        self.cur_goal.target_pose.pose.position.x = goal.pos.position.x
        self.cur_goal.target_pose.pose.position.y = goal.pos.position.y
        self.cur_goal.target_pose.pose.position.z = 0 
        self.cur_goal.target_pose.pose.orientation.z = goal.pos.orientation.z
        self.cur_goal.target_pose.pose.orientation.w = goal.pos.orientation.w
        # 监听global_planner
        self.gp_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan',Path,updata_global_plan,queue_size=10)
        self.ac.send_goal(self.cur_goal,move_base_done_cb,move_base_active_cb,move_base_fb_cb)  
        self.reach_flag = False
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
        self.gp_sub.unregister()
        rospy.loginfo("导航结束！")

    # 监听速度，实现动态避障
    def slow_detect(self,msg): 
        def move_base_done_cb(state,result):
            if state == actionlib.GoalStatus.SUCCEEDED:
                self.reach_flag = True
            else :
                self.reach_flag = True
                self.fail = True

        def move_base_active_cb():
            rospy.loginfo("move_base服务被激活....")


        def move_base_fb_cb(fb):
            if self.plan_init == False:
                self.feedback.cur_state = "planning"
                self.feedback.percentage = 0
                self.server.publish_feedback(self.feedback) 
                return
            self.feedback.cur_state = "normal"
            x = fb.base_position.pose.position.x
            y = fb.base_position.pose.position.y 
            z = fb.base_position.pose.orientation.z
            # 计算进度
            min_dis = None
            tar = None
            for i in range(self.cur_pos_id,len(self.global_plan)):
                dis = pow((x-self.global_plan[i][0]),2) + pow(y-self.global_plan[i][1],2) + 0.1 * abs(z-self.global_plan[i][2])
                if tar == None or dis < min_dis:
                    min_dis = dis
                    tar = i
                    

            self.cur_pos_id = tar
            self.feedback.percentage = int(100 * float(self.cur_pos_id + 1) / len(self.global_plan))
            self.server.publish_feedback(self.feedback)  

        if self.plan_init == False:
            return
        linear = pow(msg.linear.x,2) + pow(msg.linear.y,2)  
        angular = abs(msg.angular.z)
        # 如果速度多次低于阈值，则重新设置航点
        # rospy.loginfo(rospy.loginfo("slow_cnt={}".format(self.slow_cnt)))
        # rospy.loginfo("reach_flag={}".format(self.reach_flag)) 
        if self.slow_cnt == 2 and self.reach_flag == False and self.plan_init == True:
            self.slow_cnt = 0
            self.plan_init = False  
            self.global_plan = None
            self.cur_pos_id = 0
            rospy.logwarn("机器人速度过慢，应该是遇到障碍，正在重新设置航点，新路径计算中...") 
            self.feedback.cur_state = "barrier"
            self.feedback.percentage = 0
            self.server.publish_feedback(self.feedback) 
             
            
            self.ac.cancel_goal()  # 取消当前目标
            self.ac.send_goal(self.cur_goal,move_base_done_cb,move_base_active_cb,move_base_fb_cb)
               
    
        
        if linear  < LINEAR_THRESHOLD and angular < ANGULAR_THRESHOLD and self.reach_flag == False and self.plan_init == True:  
            self.slow_cnt += 1
            rospy.sleep(1)
        else :
            self.slow_cnt = 0
    

rospy.init_node("simple_goal")  
server = SimpleGoalServer()
rospy.spin()






