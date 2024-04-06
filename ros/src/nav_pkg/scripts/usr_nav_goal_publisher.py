#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String

def call_back(msg):
    print("usr get response")

if __name__ == "__main__":
    rospy.init_node("usr_nav_goal_publisher")
    pub = rospy.Publisher("/usr_nav_goal",String,queue_size=10)

    msg = String()  #创建 msg 对象


    x = float(input("输入航点x="))
    y = float(input("输入航点y="))
    str = str(x) + " " + str(y) + " " + "0"
    msg.data = str

    pub.publish(msg)

    sub = rospy.Subscriber("/usr_nav_result",String,call_back)

    rospy.spin()


    


        
