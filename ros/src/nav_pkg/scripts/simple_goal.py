#!/usr/bin/env python  
# coding=utf-8  
import rospy  
import actionlib  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from geometry_msgs.msg import Twist  
from std_msgs.msg import String
import time  

# 设置速度阈值  
LINEAR_THRESHOLD = 0.00001  # 例如，0.1 m/s  
ANGULAR_THRESHOLD = 0.00001  
# 低速异常计数器
slow_cnt = 0   
goal = None
ac = None
reach_flag = False

def done_cb(state, result):  
    if state == actionlib.GoalStatus.SUCCEEDED:  
        rospy.loginfo("导航成功")  
    elif state == actionlib.GoalStatus.ABORTED:  
        rospy.loginfo("导航被中止")  
    # ... 可以添加其他状态的处理 ...  
  
def active_cb():  
    rospy.loginfo("导航开始...")  


def callback(msg): 
    global slow_cnt,goal,ac,reach_flag
    linear = pow(msg.linear.x,2) + pow(msg.linear.y,2)  
    angular = abs(msg.angular.z)
    # 如果速度多次低于阈值，则重新设置航点  
    if slow_cnt == 2 and reach_flag == False:
        slow_cnt = 0
        rospy.logwarn("机器人速度过慢，应该是遇到障碍，正在重新设置航点，新路径计算中...")  
        ac.cancel_goal()  # 取消当前目标  
        ac.send_goal(goal, done_cb, active_cb)   
  
        # 这里可以重新发送目标，但注意避免无限循环
        
        # 你可能需要实现一些逻辑来确保不会频繁地重置目标 
    
    if linear  < LINEAR_THRESHOLD and angular < ANGULAR_THRESHOLD and reach_flag == False:  
        slow_cnt += 1
    else :
        slow_cnt = 0

def start_nav(msg):
    global ac,goal,reach_flag
    ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)  
    ac.wait_for_server()  
    # 处理String 类型消息
    x,y,z = msg.data.split()
    goal = MoveBaseGoal()  
    goal.target_pose.header.frame_id = "map"  
    goal.target_pose.pose.position.x = float(x) 
    goal.target_pose.pose.position.y = float(y)  
    goal.target_pose.pose.position.z = float(z)  
    goal.target_pose.pose.orientation.w = 1.0  
    ac.send_goal(goal, done_cb, active_cb)  
    rospy.loginfo("开始导航……")    
    # 在这里调用函数来监控速度并可能重新设置航点  
    vel_sub = rospy.Subscriber('/cmd_vel',Twist, callback)  

    ac.wait_for_result()
    reach_flag = True
    slow_cnt = 0
      
    rospy.loginfo("导航结束！")
    result_msg = String()
    result_msg.data = "Finish"
    result_pub.publish(result_msg)

           
if __name__ == "__main__":  
    rospy.init_node("simple_goal")  

    # 订阅nav_goal话题
    goal_sub = rospy.Subscriber('/usr_nav_goal',String ,start_nav,queue_size=10)
    result_pub = rospy.Publisher("/usr_nav_result",String)

    rospy.spin()




