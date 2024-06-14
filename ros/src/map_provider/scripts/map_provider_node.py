#!/usr/bin/env python
# coding=utf-8

import os
import rospy, roslaunch
import shutil
import subprocess

import actionlib

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String, Header
from map_provider.msg import *              # action 也包含在内
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseStamped, PointStamped, PoseWithCovarianceStamped


cur_dir = os.path.dirname(os.path.abspath(__file__))    # 当前文件夹
pkg_dir = os.path.dirname(cur_dir)                      # map_provider 包目录
maps_dir = os.path.join(pkg_dir, 'maps')                # map_provider/maps，用于存放地图

class MapProviderNode:
    def __init__(self):
        rospy.init_node('map_provider_node')

        # 订阅主题
        rospy.Subscriber('/map_provider/save_map', SaveMap, self.save_map_callback)
        rospy.Subscriber('/map_provider/clear_map', ClearMap, self.clear_map_callback)
        rospy.Subscriber('/map_provider/set_position', SetPosition, self.set_position_callback)
        rospy.Subscriber('/map_provider/manual_init_map_start_msg', ManualInitMapStart, self.manual_init_map_start_callback)
        rospy.Subscriber('/map_provider/manual_init_map_stop_msg', ManualInitMapStop, self.manual_init_map_stop_callback)
        self.initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        # 发布主题
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.point_pub = rospy.Publisher('/rrt_publish_point', PointStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Action servers
        self.auto_map_server = actionlib.SimpleActionServer('/map_provider/auto_init_map', InitMapAction, self.auto_map, False)
        self.manual_map_server = actionlib.SimpleActionServer('/map_provider/manual_init_map', InitMapAction, self.manual_map, False)
        # self.manual_map_server.register_preempt_callback(self.manual_preeme_cb)

        # subprocess
        self.slam_process = None        # 用于存储 SLAM 进程的引用
        self.map_server_process = None  # map_server 进程
        self.amcl_process = None        # amcl 进程
        self.rrt_process = None         # rrt 进程
        self.assigner_process = None    # rrt.assigner 进程

        self.auto_map_server.start()
        self.manual_map_server.start()
        rospy.loginfo("Map Provider Node has started.")

        # 检查 maps_dir 下是否存放有 .pgm 和同名 .yaml 文件，有的话直接用 map_server 读取
        self.check_and_launch_map_server()

    # 检查 maps_dir 下是否存放有 .pgm 和同名 .yaml 文件，有的话直接用 map_server 读取
    def check_and_launch_map_server(self):
        rospy.loginfo("[map_provider] Checking for existing maps in %s", maps_dir)
        if not os.path.exists(maps_dir):
            rospy.logwarn("[map_provider] Maps directory does not exist: %s", maps_dir)
            return
        
        map_files = [f for f in os.listdir(maps_dir) if os.path.isfile(os.path.join(maps_dir, f))]
        pgm_files = [f for f in map_files if f.endswith('.pgm')]
        
        # 只做一次扫描，有同名 .pgm & .yaml 文件即加载
        for pgm_file in pgm_files:
            yaml_file = pgm_file.replace('.pgm', '.yaml')
            if yaml_file in map_files:
                rospy.loginfo("[map_provider] Found map pair: %s and %s", pgm_file, yaml_file)
                map_path = os.path.join(maps_dir, pgm_file)
                yaml_path = os.path.join(maps_dir, yaml_file)
                self.launch_map_server(yaml_path)
                return
            else:
                rospy.logwarn("[map_provider] Found .pgm without matching .yaml file: %s", pgm_file)
        rospy.loginfo("[map_provider] No maps found in %s", maps_dir)

    def launch_map_server(self, map_yaml_path):
        rospy.loginfo("[map_provider] Launching map_server with %s", map_yaml_path)
        map_server_command = "rosrun map_server map_server {} __name:=map_server".format(map_yaml_path)
        try:
            self.map_server_process = subprocess.Popen(map_server_command, shell=True)
            rospy.loginfo("[map_provider] Map server launched.")
        except Exception as e:
            rospy.logerr("Failed to launch map server for %s: %s", map_yaml_path, e)
            self.map_server_process = None

    # 0520 - new
    def save_map(self, map_name):
        rospy.loginfo("[map_provider - save_map] Saving map...")
        save_target_path = os.path.join(maps_dir, map_name)
        
        try:
            rospy.loginfo("[map_provider - save_map] before...")
            retcode = subprocess.call(['rosrun', 'map_server', 'map_saver', '-f', save_target_path])
            rospy.loginfo("[map_provider - save_map] after...")

            # 终止 /slam_gmapping
            subprocess.check_call(["rosnode", "kill", "/slam_gmapping"])
            rospy.sleep(1)

            self.check_and_launch_map_server()  # 启动 map_server

            if retcode == 0:
                rospy.loginfo("[map_provider - save_map] Map saved successfully")
                rospy.loginfo("[map_provider - save_map] Map has been saved to {}".format(save_target_path))
            else:
                rospy.logerr("[map_provider - save_map] Failed to save map, command returned code: {}".format(retcode))
        except Exception as e:
            rospy.logerr("[map_provider - save_map] Error occurred when trying to save the map: %s", e)

    # 保存地图的回调函数
    def save_map_callback(self, data):
        rospy.loginfo("[map_provider - save_map_callback] Saving map...")
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

    def kill_map_server(self):
        rospy.loginfo("Killing map_server...")
        try:
            # 使用 rosnode kill 命令终止 map_server 节点
            subprocess.check_call(["rosnode", "kill", "/map_server"])
            rospy.loginfo("map_server has been successfully killed.")
        except subprocess.CalledProcessError as e:
            rospy.logerr("Failed to kill map_server: %s", e)

    # 清理地图的回调函数 + 关闭 /map_server
    # 实现：清空 maps_dir 下的所有文件
    # data 暂时用不到
    def clear_map_callback(self, data):
        rospy.loginfo("Clearing map...")
        for _ in range(3):  # 重复清除地图三次
            rospy.loginfo("Attempt to clear all maps in the directory {}.".format(maps_dir))

            # 检查目录是否存在
            if not os.path.isdir(maps_dir):
                rospy.logerr("The directory %s does not exist.", maps_dir)
                break

            try:
                # 清除地图前先关闭地图服务器
                self.kill_map_server()

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

                # 检查是否还有文件，即目录是否已清空
                if not os.listdir(maps_dir):
                    rospy.loginfo("All maps in %s have been successfully deleted.", maps_dir)
                    break  # 如果目录已空，退出循环
                else:
                    rospy.logwarn("Maps directory is not empty after deletion attempt.")
            except Exception as e:
                rospy.logerr("Error occurred while deleting maps: %s", e)
                break  # 遇到异常时退出循环

    # 手动设定位置的回调函数
    # data 是 geometry_msgs/Pose pos，yaml 格式的 data 如下（示例见test_set_pos.yaml）：
    """
    pos:
    position:
        x: 1.0
        y: 1.0
        z: 2.0
    orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 10.0
    """
    def set_position_callback(self, data):
        rospy.loginfo("Setting initial robot position...")
        # 创建一个带协方差的位姿消息，这通常用于设置初始位姿
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"  # 或者是地图的 frame_id，如果它被重命名了
        initial_pose_msg.pose.pose = data.pos  # 使用从 SetPosition 消息接收到的位姿
        # 这里可以设置协方差，但是通常默认值就足够了
        # 0609更新：设置协方差矩阵默认为 0.12I
        initial_pose_msg.pose.covariance = [0.0] * 36
        initial_pose_msg.pose.covariance[0] = 0.12      # x 方向的位置协方差
        initial_pose_msg.pose.covariance[7] = 0.12      # y 方向的位置协方差
        initial_pose_msg.pose.covariance[14] = 0.12     # z 方向的位置协方差
        initial_pose_msg.pose.covariance[21] = 0.12     # x 方向的姿态协方差
        initial_pose_msg.pose.covariance[28] = 0.12     # y 方向的姿态协方差
        initial_pose_msg.pose.covariance[35] = 0.12     # z 方向的姿态协方差

        # 发布初始化位姿到 /initialpose 主题，让导航堆栈知道机器人在地图上的初始位置
        self.initial_pose_pub.publish(initial_pose_msg)
        rospy.loginfo("Robot initial position has been set.")

    # 自动建图的回调函数
    # TODO: 实现自动建图逻辑
    def auto_map(self, goal):
        rospy.loginfo("[map_provider - auto_map] Starting automatic mapping...")
        # 示例的进度反馈和结果发布
        feedback = InitMapFeedback()
        feedback.percentage = -1    # 简化版本，不关心进度
        self.auto_map_server.publish_feedback(feedback)

        # todo_0: 关闭 /map_server 和 /amcl, 启动 /slam_gmapping
        subprocess.call(["rosnode", "kill", "/map_server"])
        subprocess.call(["rosnode", "kill", "/amcl"])
        self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'dev-slam_gmapping.launch'])
        rospy.loginfo("[map_provider - auto_map] SLAM process started.")

        # todo_1: 启动 rrt_start.launch & rrt_assigner.launch
        try:
            rospy.loginfo("[map_provider - auto_map] Launching rrt_start.launch...")
            self.rrt_process = subprocess.Popen(['roslaunch', 'map_provider', 'rrt_start.launch'])

            rospy.sleep(3)

            rospy.loginfo("[map_provider - auto_map] rrt_start.launch launched successfully.")
            self.assigner_process = subprocess.Popen(['roslaunch', 'map_provider', 'rrt_assigner.launch'])
            rospy.loginfo("[map_provider - auto_map] rrt_assigner.launch launched successfully.")
        except Exception as e:
            rospy.logerr("[map_provider - auto_map] Failed to launch rrt: %s", e)
            self.auto_map_server.set_aborted(InitMapResult(result='error'))
            return

        # 休眠，等待 rrt 启动
        rospy.loginfo("[map_provider - auto_map] sleep, waiting for rrt starting")
        rospy.sleep(1.5)
        rospy.loginfo("[map_provider - auto_map] awoken")

        # todo_2: 向 /rrt_publish_point 发布四个边界点
        points = [
            (-3.0, 3.0, 0.0),
            (4.0, 3.0, 0.0),
            (4.0, -3.0, 0.0),
            (-3.0, -3.0, 0.0)
        ]
        for x, y, z in points:
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "map"
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z
            self.point_pub.publish(point_msg)
            rospy.loginfo("[map_provider - auto_map] Published point: (%s, %s, %s)", x, y, z)
            rospy.sleep(0.1)  # 确保每个点都被正确发布

        # todo_3: 向 /rrt_publish_point 发布一个初始目标点
        goal_point = (1.0, 1.0, 0.0)
        goal_msg = PointStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.point.x = goal_point[0]
        goal_msg.point.y = goal_point[1]
        goal_msg.point.z = goal_point[2]
        self.point_pub.publish(goal_msg)
        rospy.loginfo("[map_provider - auto_map] Published goal point: %s", goal_point)

        # 休眠，等待航点
        rospy.loginfo("[map_provider - auto_map] sleep, waiting for move_base_simple/goal")
        rospy.sleep(1.5)
        rospy.loginfo("[map_provider - auto_map] awoken")

        # todo_4: 向 /move_base_simple/goal 发布一个初始目标点
        def auto_map_send_nav_goal(x, y, z, w, save_map):
            # 创建一个SimpleActionClient，使用move_base的MoveBaseAction接口
            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            # 取消当前的导航目标
            rospy.loginfo("[map_provider - auto_map] Cancelling any existing goals...")
            client.cancel_all_goals()

            # 创建目标点
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            # 设置目标位置
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.position.z = z

            # 设置目标方向（四元数）
            goal.target_pose.pose.orientation.x = 0.0
            goal.target_pose.pose.orientation.y = 0.0
            goal.target_pose.pose.orientation.z = 0.0
            goal.target_pose.pose.orientation.w = w

            rospy.loginfo("[map_provider - auto_map] First goal (%s, %s, %s, %s) to /move_base", x, y, z, w)
            client.send_goal(goal)
        
        # auto_map_send_nav_goal(1.0, 1.0, 0.0, 1.0, save_map=False)
        auto_map_send_nav_goal(0.0, -1.0, 0.0, 1.0, save_map=False)

        while not rospy.is_shutdown():
            # rospy.loginfo('[map_provider - auto_map] auto map in progress...')
            # rospy.loginfo('[map_provider - auto_map] subprocess assigner_process.launch: %s, poll(): %s', self.assigner_process, self.assigner_process.poll())
            
            # 检查 assigner 进程是否已经结束
            if self.assigner_process.poll() is not None:
                rospy.loginfo('[map_provider - auto_map] assigner_process.launch process has finished cleanly.')
                try:
                    # 保存地图
                    self.save_map('saved_map')
                    rospy.loginfo("[map_provider - auto_map] Auto map completed successfully.")

                    # 重启 amcl
                    subprocess.Popen(['roslaunch', 'map_provider', 'amcl_omni.launch'])

                    # 终止 rrt 进程
                    if self.rrt_process is not None:
                        rospy.loginfo("[map_provider - auto_map] Terminating rrt_process...")
                        self.rrt_process.terminate()
                        try:
                            self.rrt_process.wait(timeout=5)  # 等待进程结束，最多等待5秒
                            rospy.loginfo("[map_provider - auto_map] rrt_process terminated gracefully.")
                        except Exception:
                            rospy.logwarn("[map_provider - auto_map] rrt_process did not terminate gracefully, killing it.")
                            self.rrt_process.kill()
                            self.rrt_process.wait()
                        self.rrt_process = None
                    else:
                        rospy.loginfo("[map_provider - auto_map] No rrt_process to terminate.")

                    result = InitMapResult(result='success')
                    self.auto_map_server.set_succeeded(result)
                except subprocess.CalledProcessError as e:
                    rospy.logerr("Failed to save map: %s", e)
                    result = InitMapResult(result='error')
                    self.auto_map_server.set_aborted(result)
                break

            if self.auto_map_server.is_preempt_requested():
                rospy.loginfo('[map_provider - auto_map] Preempted auto map')
                result = InitMapResult(result='cancel')
                try:
                    self.save_map('saved_map')
                    rospy.loginfo("[map_provider - auto_map] Auto map canceled, map saved.")
                except subprocess.CalledProcessError as e:
                    rospy.logerr("Failed to save map: %s", e)
                    result.result = 'error'
                finally:
                    self.auto_map_server.set_preempted(result)
                break

            rospy.sleep(2)  # 休眠，以避免过度占用CPU
        
        rospy.loginfo("[map_provider] Successfully completed automatic mapping.")

    # 0520 - new
    def manual_map(self, goal):
        rospy.loginfo("Received manual mapping request, starting manual mapping...")
        feedback = InitMapFeedback()
        feedback.percentage = -1  # 简化版本，不关心进度
        self.manual_map_server.publish_feedback(feedback)

        if self.slam_process is None:
            try:
                # 终止 /map_server 和 /amcl
                subprocess.call(["rosnode", "kill", "/map_server"])
                subprocess.call(["rosnode", "kill", "/amcl"])
                # 启动SLAM进程
                self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'dev-slam_gmapping.launch'])
                rospy.loginfo("SLAM process started.")
            except Exception as e:
                rospy.logerr("Launching SLAM process failed: %s", e)
                result = InitMapResult(result='error')
                self.manual_map_server.set_aborted(result)
                return
        else:
            rospy.logwarn("SLAM process is already running.")
            result = InitMapResult(result='error')
            self.manual_map_server.set_aborted(result)
            return

        # 在这个 while 循环中等待 cancel 请求
        while not rospy.is_shutdown():
            # rospy.loginfo('[ManualMap] into while')
            if self.manual_map_server.is_preempt_requested():
                rospy.loginfo('Preempted Manual Map')
                result = InitMapResult(result='success')
                try:
                    # Save the map once mapping is complete.
                    self.save_map('saved_map')
                    # 使用rosnode kill命令结束slam_gmapping节点
                    # subprocess.check_call(["rosnode", "kill", "/slam_gmapping"])
                    # 重启 amcl
                    subprocess.Popen(['roslaunch', 'map_provider', 'amcl_omni.launch'])
                    rospy.loginfo("SLAM process has been terminated using rosnode kill.")
                except subprocess.CalledProcessError as e:
                    rospy.logerr("Failed to terminate SLAM process with rosnode kill: %s", e)
                    result.result = 'error'
                finally:
                    # self.save_map('saved_map')
                    rospy.loginfo("SLAM process completed and map has been saved.")
                    self.slam_process = None  # Reset the SLAM process variable
                    self.manual_map_server.set_preempted(result)  # 设置操作已被取消的结果
                    # self.check_and_launch_map_server()  # 启动 map_server
                break
            rospy.sleep(1)  # 休眠，以避免过度占用CPU
        
        rospy.loginfo("[map_provider] Successfully completed manual mapping.")
        # result = InitMapResult(result='success')
        # self.manual_map_server.set_succeeded(result)

    # 手动建图 preempt 时的处理函数，但不知道为什么触发不了
    def manual_preeme_cb(self):
        rospy.loginfo("Manual mapping has been preempted/cancelled.")
        result = InitMapResult()
        if self.slam_process:
            try:
                # Save the map once mapping is complete.
                self.save_map('saved_map')
                # 使用rosnode kill命令结束slam_gmapping节点
                self.slam_process.terminate()
                rospy.loginfo("SLAM process has been terminated.")
                result.result = 'success'
                self.slam_process = None

                # 启动 map_server
                self.check_and_launch_map_server()

                # 启动 amcl 
                subprocess.Popen(['roslaunch', 'map_provider', 'amcl_omni.launch'])
            except Exception as e:
                rospy.logerr("Failed to terminate SLAM process: %s", e)
                result.result = 'fail'
        else:
            rospy.loginfo("No active SLAM process to cancel.")
            result.result = 'fail'
        self.manual_map_server.set_succeeded(result)  # 设置操作已被取消的结果
    
    # ====================================== deprecated ====================================== #
    # 尝试使用 Msg 机制来触发手动建图
    # 手动建图本质：启动 slam 与 关闭 slam
    # 启动/停止 slam 进程即返回
    # Msg 无 result 返回
    def manual_init_map_start_callback(self, data):
        rospy.loginfo("manual_init_map_start_callback called by {}".format(data.caller))
        # 检查 /slam_gmapping 节点是否开启
        try:
            # 等同于 `rosnode list` 命令
            nodes = subprocess.check_output(["rosnode", "list"]).decode('utf-8').strip().split("\n")
            if '/slam_gmapping' not in nodes:
                rospy.loginfo("/slam_gmapping node is not running, starting it...")
                self.slam_process = subprocess.Popen(['roslaunch', 'map_provider', 'dev-slam_gmapping.launch'])
            else:
                rospy.loginfo("/slam_gmapping node already running.")
        except Exception as e:
            rospy.logerr("An error occurred while checking for the /slam_gmapping node: %s", e)

    def manual_init_map_stop_callback(self, data):
        rospy.loginfo("manual_init_map_stop_callback called by {}".format(data.caller))
        # 检查 /slam_gmapping 节点是否开启
        try:
            # 等同于 `rosnode list` 命令
            nodes = subprocess.check_output(["rosnode", "list"]).decode('utf-8').strip().split("\n")
            if '/slam_gmapping' in nodes:
                rospy.loginfo("/slam_gmapping node is running, stopping it...")
                subprocess.call(["rosnode", "kill", "/slam_gmapping"])
            else:
                rospy.loginfo("/slam_gmapping node is not running, nothing to stop.")
        except Exception as e:
            rospy.logerr("An error occurred while attempting to kill the /slam_gmapping node: %s", e)

if __name__ == '__main__':
    try:
        map_provider_node = MapProviderNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
