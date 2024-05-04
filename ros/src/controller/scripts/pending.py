#! /usr/bin/env python
# coding=utf-8 


import rospy
from controller.msg import Hello, NodeInfo
from controller.srv import *
import actionlib
from navigation.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped


class PendingServer():
    def __init__(self):
        # 与前端交互节点状态
        def hello_callback(msg):
            rospy.loginfo("Received hello message")
            self.node_info_pub.publish(self.node_info)

        self.node_info = NodeInfo() # 核心状态变量
        
        self.node_info.state = self.node_info.Stop
        self.node_info.mode = 1 # 模式是pending

        # 与前端交互节点状态
        self.hello_sub = rospy.Subscriber('/hello', Hello, hello_callback)
        self.node_info_pub = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        self.node_info_pub.publish(self.node_info)

        
        self.start_server = rospy.Service("/ctrl/pending/start",Start,self.start_serve)
        self.stop_server = rospy.Service("/ctrl/pending/stop",Stop,self.stop_serve)
        # 五种服务启动
        self.clear_map_server = rospy.Service("/ctrl/pending/clear_map",ClearMap,self.clear_map_serve)
        self.auto_init_pos_server = rospy.Service("/ctrl/pending/auto_init_pos",AutoInitPos,self.auto_init_pos_serve)
        self.manual_init_pos_server = rospy.Service("/ctrl/pending/manual_init_pos",ManualInitPos,self.manual_init_pos_serve)
        self.save_map_server = rospy.Service("/ctrl/pending/save_map",SaveMap,self.save_map_serve)
        self.navigate_server = actionlib.SimpleActionServer("/ctrl/pending/navigate",NavigateAction,self.navigate_serve,False)
        self.navigate_server.start()
        rospy.spin()

    def is_started(self):
        return self.node_info.state == self.node_info.Wait
        
    # 改状态至Wait,pending只有在Wait的时候才接受五种服务
    def start_serve(self,req):
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)
        resp = StartResponse()
        resp.success = True
        return resp

    # 改状态至Stop
    def stop_serve(self,req):
        self.node_info.state = self.node_info.Stop
        self.node_info_pub.publish(self.node_info)
        resp = StopResponse()
        resp.success = True
        return resp
    
    def clear_map_serve(self,req):
        if not self.is_started():
            resp = ManualInitPosResponse()
            resp.success = False
            return resp
        self.node_info.state = self.node_info.Clear_Map
        self.node_info_pub.publish(self.node_info)
        rospy.loginfo("假装清除了已保存的地图")
        resp = ClearMapResponse()
        resp.success = True
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)
        return resp

    def auto_init_pos_serve(self,req):
        if not self.is_started():
            resp = ManualInitPosResponse()
            resp.success = False
            return resp
        self.node_info.state = self.node_info.Auto_Init_Map
        self.node_info_pub.publish(self.node_info)
        rospy.loginfo("假装自动设置了机器人初始位置")
        resp = AutoInitPosResponse()
        resp.success = True
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)
        return resp

    def manual_init_pos_serve(self,req):
        if not self.is_started():
            resp = ManualInitPosResponse()
            resp.success = False
            return resp
        self.node_info.state = self.node_info.Manual_Init_Map
        self.node_info_pub.publish(self.node_info)
        rospy.loginfo("我真的手动设置了机器人初始位置")
        init_pos_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "map"
        p.pose.pose.position.x = req.pos.position.x
        p.pose.pose.position.y = req.pos.position.y
        p.pose.pose.position.z = 0
        p.pose.pose.orientation.z = req.pos.orientation.z
        p.pose.pose.orientation.w = req.pos.orientation.w
        p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        p.pose.covariance[35] =  0.06853892326654787
        init_pos_pub.publish(p)
        rospy.sleep(1)
        init_pos_pub.publish(p)
        rospy.sleep(1)
        resp = ManualInitPosResponse()
        resp.success = True
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)
        return resp

    def save_map_serve(self,req):
        if not self.is_started():
            resp = ManualInitPosResponse()
            resp.success = False
            return resp
        self.node_info.state = self.node_info.SaveMap
        self.node_info_pub.publish(self.node_info)
        rospy.loginfo("假装保存了地图")
        resp = SaveMapResponse()
        resp.success = True
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)
        return resp

    def navigate_serve(self,goal):
        if not self.is_started():
            result = NavigateResult()
            result.result = "fail"
            self.navigate_server.set_succeeded(result)
            return
        self.end_flag = False
        self.result = NavigateResult()
        def done_cb(state,r):
            self.result.result = r.result
            self.end_flag = True
                    
        def active_cb():
            rospy.loginfo("服务被激活....")
                
        def fb_cb(fb):
            feedback = NavigateFeedback()
            feedback.percentage = fb.percentage
            feedback.cur_state = fb.cur_state
            rospy.loginfo("{}".format(feedback.percentage))
            rospy.loginfo("当前进度：{}".format(feedback.cur_state))
            self.navigate_server.publish_feedback(feedback) 


        self.node_info.state = self.node_info.Navigate
        self.node_info_pub.publish(self.node_info)
        client = actionlib.SimpleActionClient("navigation/navigate",NavigateAction)
        client.wait_for_server()
        client.send_goal(goal,done_cb,active_cb,fb_cb)
        rate = rospy.Rate(1)
            
        while not rospy.is_shutdown():
            if self.navigate_server.is_preempt_requested(): # cancel了
                client.cancel_goal()
                self.result.result = "cancel"
                self.navigate_server.set_succeeded(self.result)
                break
            if self.end_flag == True: # move_base导航失败
                self.navigate_server.set_succeeded(self.result)
                break
            rate.sleep()
        rospy.loginfo("导航结束！")

                    
        self.node_info.state = self.node_info.Wait
        self.node_info_pub.publish(self.node_info)






        




if __name__ == "__main__":
    rospy.init_node("pending")
    pending_server = PendingServer()
    
    
    
