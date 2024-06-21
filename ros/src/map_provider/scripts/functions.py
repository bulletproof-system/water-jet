#!/usr/bin/env python
# coding=utf-8

import tf
import rospy
import actionlib

from numpy import array
from numpy import inf, floor
from numpy.linalg import norm

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped


class Robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param(
            '~plan_service', '/move_base/GlobalPlanner/make_plan'
        )
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(
            self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('[map_provider - auto_map - functions.py] Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient(
            self.name + '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        Robot.goal.target_pose.header.frame_id = 'map'
        Robot.goal.target_pose.header.stamp = rospy.Time.now()

        rospy.wait_for_service(self.name + self.plan_service)
        self.make_plan = rospy.ServiceProxy(
            self.name + self.plan_service, GetPlan)
        Robot.start.header.frame_id = self.global_frame
        Robot.end.header.frame_id = self.global_frame

    def get_position(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond == 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def send_nav_goal(self, point, exit=False, escape=False):
        if not self.is_reachable(self.get_position(), point) and not escape:
            rospy.logwarn("[map_provider - auto_map - functions.py] Goal is not reachable: %s", point)
            return False

        Robot.goal.target_pose.pose.position.x = point[0]
        Robot.goal.target_pose.pose.position.y = point[1]
        Robot.goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(Robot.goal)
        self.assigned_point = array(point)

        if exit:
            self.client.wait_for_result()
            state = self.client.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("[map_provider - auto_map - functions.py] Navigation to (0, 0, 0, 0) succeeded!")
                return True
            else:
                rospy.logwarn("[map_provider - auto_map - functions.py] Navigation failed with state: %s", state)
                return False
        else:
            return True

    def cancel_nav_goal(self):
        self.client.cancel_goal()
        self.assigned_point = self.get_position()

    def get_state(self):
        return self.client.get_state()

    def make_plan(self, start, end):
        Robot.start.pose.position.x = start[0]
        Robot.start.pose.position.y = start[1]
        Robot.end.pose.position.x = end[0]
        Robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name + 'map', Robot.start)
        end = self.listener.transformPose(self.name + 'map', Robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        # rospy.loginfo("[map_provider - auto_map - functions.py] maked plan")
        return plan.plan.poses

    def is_reachable(self, start, end):
        # 检查从 start 到 end 是否有可行路径
        plan = self.make_plan(start, end)
        if len(plan) > 0:
            return True
        return False

    def get_nearby_reachable_points(self, radius=0.5):
        directions = [
            [0, radius], [0, -radius], [radius, 0], [-radius, 0],  # 前后左右
            [radius, radius], [radius, -radius], [-radius, radius], [-radius, -radius]  # 左上右上左下右下
        ]
        current_position = self.get_position()
        for direction in directions:
            potential_point = current_position + array(direction)
            if self.is_reachable(current_position, potential_point):
                return potential_point
        return None


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int((floor((Xp[1] - Xstarty) / resolution) *
                 width) + (floor((Xp[0] - Xstartx) / resolution)))
    return index


def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i / mapData.info.width) * mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i - (i // mapData.info.width) * (mapData.info.width)) * mapData.info.resolution
    return array([x, y])


def informationGain(mapData, point, r):
    infoGain = 0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == -1 and norm(array(point) - point_of_index(mapData, i)) <= r):
                    infoGain += 1
    return infoGain * (mapData.info.resolution ** 2)


def discount(mapData, assigned_pt, centroids, infoGain, r):
    index = index_of_point(mapData, assigned_pt)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                for j in range(0, len(centroids)):
                    current_pt = centroids[j]
                    if (mapData.data[i] == -1 and \
                            norm(point_of_index(mapData, i) - current_pt) <= r and \
                            norm(point_of_index(mapData, i) - assigned_pt) <= r):
                        # this should be modified, subtract the area of a cell, not 1
                        infoGain[j] -= 1
    return infoGain


def pathCost(path):
    if (len(path) > 0):
        i = len(path) / 2
        p1 = array([path[i - 1].pose.position.x, path[i - 1].pose.position.y])
        p2 = array([path[i].pose.position.x, path[i].pose.position.y])
        return norm(p1 - p2) * (len(path) - 1)
    else:
        return inf


def unvalid(mapData, pt):
    index = index_of_point(mapData, pt)
    r_region = 5
    init_index = index - r_region * (mapData.info.width + 1)
    for n in range(0, 2 * r_region + 1):
        start = n * mapData.info.width + init_index
        end = start + 2 * r_region
        limit = ((start / mapData.info.width) + 2) * mapData.info.width
        for i in range(start, end + 1):
            if (i >= 0 and i < limit and i < len(mapData.data)):
                if (mapData.data[i] == 1):
                    return True
    return False


def Nearest(V, x):
    n = inf
    i = 0
    for i in range(0, V.shape[0]):
        n1 = norm(V[i, :] - x)
        if (n1 < n):
            n = n1
            result = i
    return result


def Nearest2(V, x):
    n = inf
    for i in range(0, len(V)):
        n1 = norm(V[i] - x)
        if (n1 < n):
            n = n1
    return i


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # 100: occupied
    # -1: unknown
    # 0: free
    index = (floor((Xp[1] - Xstarty) / resolution) * width) + (floor((Xp[0] - Xstartx) / resolution))

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
