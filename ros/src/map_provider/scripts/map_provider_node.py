#!/usr/bin/env python
# coding=utf-8

import os
import time
import rospy
import shutil
import subprocess

import actionlib
from std_msgs.msg import String
from map_provider.msg import *
# from map_provider.action import InitMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


cur_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(cur_dir)
maps_dir = os.path.join(pkg_dir, 'maps')

class MapProviderNode:
    def __init__(self):
        rospy.init_node('map_provider_node')

        # 订阅主题
        rospy.Subscriber('/map_provider/save_map', SaveMap, self.save_map_callback)
        rospy.Subscriber('/map_provider/clear_map', ClearMap, self.clear_map_callback)
        rospy.Subscriber('/map_provider/set_position', SetPosition, self.set_position_callback)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # 发布主题
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)

        # Action servers
        self.auto_map_server = actionlib.SimpleActionServer('/map_provider/auto_init_map', InitMapAction, self.auto_map, False)
        self.manual_map_server = actionlib.SimpleActionServer('/map_provider/manual_init_map', InitMapAction, self.manual_map, False)
        self.manual_map_server.register_preempt_callback(self.manual_preeme_cb)
        self.slam_process = None        # 用于存储 SLAM 进程的引用

        self.auto_map_server.start()
        self.manual_map_server.start()
        rospy.loginfo("Map Provider Node has started.")

    def save_map_callback(self, data):
        rospy.loginfo("Saving map...")
        map_name = data.name
        save_target_path = os.path.join(maps_dir, map_name)
        save_command = "rosrun map_server map_saver -f {}".format(save_target_path)

        try:
            process = subprocess.Popen(save_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            # 等待命令执行完成
            stdout, stderr = process.communicate()

            if process.returncode == 0:
                rospy.loginfo("Map saved successfully: \n%s", stdout)
                maps_dir = os.path.join(pkg_dir, 'maps')
                rospy.loginfo("Map has been saved to {}".format(save_target_path))
            else:
                rospy.logerr("Failed to save map: \n%s", stderr)

        except Exception as e:
            rospy.logerr("Error occurred: %s", e)

    def clear_map_callback(self, data):
        rospy.loginfo("Clearing map...")
        rospy.loginfo("Clearing all maps in the directory {}.".format(maps_dir))
        # time.sleep(3)
        
        # 检查目录是否存在
        if not os.path.isdir(maps_dir):
            rospy.logerr("The directory %s does not exist.", maps_dir)
            return

        try:
            # 遍历 maps_dir 目录下的所有文件和目录，并逐个删除
            for filename in os.listdir(maps_dir):
                file_path = os.path.join(maps_dir, filename)
                try:
                    if os.path.isfile(file_path) or os.path.islink(file_path):
                        os.unlink(file_path)
                    elif os.path.isdir(file_path):
                        shutil.rmtree(file_path)  # 用于删除目录
                except Exception as e:
                    rospy.logerr("Failed to delete %s. Reason: %s", file_path, e)
            rospy.loginfo("All maps in %s have been successfully deleted.", maps_dir)
        except Exception as e:
            rospy.logerr("Error occurred while deleting maps: %s", e)

    def set_position_callback(self, data):
        rospy.loginfo("Setting initial robot position...")
        # 创建一个带协方差的位姿消息，这通常用于设置初始位姿
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"  # 或者是地图的 frame_id，如果它被重命名了
        initial_pose_msg.pose.pose = data.pos  # 使用从 SetPosition 消息接收到的位姿
        # 这里可以设置协方差，但是通常默认值就足够了
        # initial_pose_msg.pose.covariance = [...]

        # 发布初始化位姿到 /initialpose 主题，让导航堆栈知道机器人在地图上的初始位置
        self.initial_pose_pub.publish(initial_pose_msg)
        rospy.loginfo("Robot initial position has been set.")

    def auto_map(self, goal):
        # TODO: 实现自动建图逻辑
        rospy.loginfo("Starting automatic mapping...")
        # 示例的进度反馈和结果发布
        feedback = InitMapFeedback()
        feedback.percentage = -1    # 简化版本，不关心进度
        self.auto_map_server.publish_feedback(feedback)
        # 一些建图逻辑...
        feedback.percentage = 100
        self.auto_map_server.publish_feedback(feedback)
        self.auto_map_server.set_succeeded(InitMapResult(result='success'))

    def manual_map(self, goal):
        rospy.loginfo("Received manual mapping request, starting manual mapping...")
        feedback = InitMapFeedback()
        feedback.percentage = -1    # 简化版本，不关心进度

        # 启动 SLAM
        print "current slam process: {}".format(self.slam_process)
        # if self.slam_process is None:
        #     try:
        #         self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'test-slam_gmapping.launch'])
        #         rospy.loginfo("SLAM process started.")
        #         result = InitMapResult(result='success')
        #         self.manual_map_server.set_succeeded(result)
        #     except Exception as e:
        #         rospy.logerr("Launching SLAM process failed: %s", e)
        #         result = InitMapResult(result='error')
        #         self.manual_map_server.set_aborted(result)
        # else:
        #     rospy.logwarn("SLAM process is already running.")
        #     # 如果 SLAM 进程已经在运行，则认为是错误
        #     result = InitMapResult(result='error')
        #     self.manual_map_server.set_aborted(result)
        # self.manual_map_server.publish_feedback(feedback)

        # if self.slam_process is None:
        #     try:
        #         self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'test-slam_gmapping.launch'])
        #         rospy.loginfo("SLAM process started.")
        #         result = InitMapResult(result='success')
        #         self.manual_map_server.set_succeeded(result)
        #     except Exception as e:
        #         rospy.logerr("Launching SLAM process failed: %s", e)
        #         result = InitMapResult(result='error')
        #         self.manual_map_server.set_aborted(result)
        # self.manual_map_server.publish_feedback(feedback)

        while not rospy.is_shutdown():
            if self.slam_process is None:
                try:
                    self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'test-slam_gmapping.launch'])
                    rospy.loginfo("SLAM process started.")
                    result = InitMapResult(result='success')
                    self.manual_map_server.set_succeeded(result)
                except Exception as e:
                    rospy.logerr("Launching SLAM process failed: %s", e)
                    result = InitMapResult(result='error')
                    self.manual_map_server.set_aborted(result)
            self.manual_map_server.publish_feedback(feedback)

            # 检查是否有预处理请求
            if self.manual_map_server.is_preempt_requested():
                rospy.loginfo('Preempted Manual Map')
                # self.manual_map_server.set_preempted()  # 通知客户端任务被预处理
                rospy.loginfo("Manual mapping has been preempted/cancelled.")
                result = InitMapResult()
                # if self.slam_process:
                #     try:
                #         self.slam_process.terminate()
                #         rospy.loginfo("SLAM process has been terminated.")
                #         result.result = 'cancel'
                #         self.slam_process = None
                #     except Exception as e:
                #         rospy.logerr("Failed to terminate SLAM process: %s", e)
                #         result.result = 'error'
                # else:
                #     rospy.loginfo("No active SLAM process to cancel.")
                #     result.result = 'cancel'
                try:
                    self.slam_process.terminate()
                    # 使用rosnode kill命令结束slam_gmapping节点
                    kill_command = "rosnode kill /slam_gmapping"
                    subprocess.call(kill_command.split(' '))
                    rospy.loginfo("SLAM process has been terminated using rosnode kill.")
                    result.result = 'cancel'
                except Exception as e:
                    rospy.logerr("Failed to terminate SLAM process with rosnode kill: %s", e)
                    result.result = 'error'
                finally:
                    self.slam_process = None
                self.manual_map_server.set_preempted(result)  # 设置操作已被取消的结果
                break
            rospy.sleep(0.1)  # 休眠，以避免过度占用CPU

        # while not rospy.is_shutdown():
        #     if self.manual_map_server.is_preempt_requested():
        #         rospy.loginfo('Preempted Manual Map')
        #         # self.manual_map_server.set_preempted()  # 通知客户端任务被预处理
        #         break
        #     rospy.sleep(0.01)  # 休眠，以避免过度占用CPU

    def manual_preeme_cb(self):
        rospy.loginfo("Manual mapping has been preempted/cancelled.")
        result = InitMapResult()
        if self.slam_process:
            try:
                self.slam_process.terminate()
                rospy.loginfo("SLAM process has been terminated.")
                result.result = 'cancel'
                self.slam_process = None
            except Exception as e:
                rospy.logerr("Failed to terminate SLAM process: %s", e)
                result.result = 'error'
        else:
            rospy.loginfo("No active SLAM process to cancel.")
            result.result = 'cancel'
        self.manual_map_server.set_preempted(result)  # 设置操作已被取消的结果

if __name__ == '__main__':
    try:
        map_provider_node = MapProviderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
