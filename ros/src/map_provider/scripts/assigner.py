#!/usr/bin/env python
# coding=utf-8

# -------- 导入模块 ---------------
import rospy
from copy import copy
from numpy import array
from numpy.linalg import norm
from collections import defaultdict
from nav_msgs.msg import OccupancyGrid
from map_provider.msg import PointArray
from functions import Robot, informationGain, discount

# 订阅者的回调函数
map_data = OccupancyGrid()
frontiers = []
global1 = OccupancyGrid()
global2 = OccupancyGrid()
global3 = OccupancyGrid()
global_maps = []


def callBack(data):
    global frontiers
    frontiers = []
    for point in data.points:
        frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
    global map_data
    mapData = data


# 节点
def node():
    global frontiers, map_data, global1, global2, global3, globalmaps
    rospy.init_node('assigner', anonymous=False)

    # 获取所有参数    
    map_topic = rospy.get_param('~map_topic', '/map')
    info_radius = rospy.get_param('~info_radius', 1.0)  # 这个值可以小于激光扫描器的范围，越小计算时间越短，但太小信息增益不准确
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)  # 至少与激光扫描器范围一样大
    hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)  # 大于1，偏向机器人继续探索当前区域
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    n_robots = rospy.get_param('~n_robots', 1)
    namespace = rospy.get_param('~namespace', '')
    namespace_init_count = rospy.get_param('namespace_init_count', 1)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.3)
    rateHz = rospy.get_param('~rate', 100)

    rate = rospy.Rate(rateHz)

    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)

    # 等待接收前沿点
    while len(frontiers) < 1:
        pass
    centroids = copy(frontiers)

    # 等待接收地图数据
    while len(mapData.data) < 1:
        pass

    # 初始化机器人
    robots = []
    if len(namespace) > 0:
        for i in range(n_robots):
            robots.append(Robot(namespace + str(i + namespace_init_count)))
    else:
        robots.append(Robot(namespace))
    for i in range(n_robots):
        robots[i].send_nav_goal(robots[i].get_position())

    # TTL字典和集合
    ttl_dict = defaultdict()  # 用于统计目标点的访问次数
    ttl_set = set()  # 用于收纳被移除的点

    # 主循环
    while not rospy.is_shutdown():
        centroids = copy(frontiers)

        # 获取每个前沿点的信息增益
        infoGain = []
        for ip in range(len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))

        # 获取可用和忙碌的机器人数量
        na = []  # 可用机器人
        nb = []  # 忙碌机器人
        for i in range(n_robots):
            if robots[i].get_state() == 1:
                nb.append(i)
            else:
                na.append(i)

        # 更新信息增益
        for i in nb + na:
            infoGain = discount(mapData, robots[i].assigned_point, centroids, infoGain, info_radius)

        revenue_record = []
        centroid_record = []
        id_record = []

        escape_mode = False
        # 在 sendGoal 之前添加可达性检查
        for ir in na:
            all_unreachable = True  # 标识位，假设所有点都不可达
            for ip in range(0, len(centroids)):
                cost = norm(robots[ir].get_position() - centroids[ip])
                information_gain = infoGain[ip]
                if (norm(robots[ir].get_position() - centroids[ip]) <= hysteresis_radius):
                    information_gain *= hysteresis_gain
                revenue = information_gain * info_multiplier - cost

                # 检查目标是否可达
                if robots[ir].is_reachable(robots[ir].get_position(), centroids[ip]):
                    all_unreachable = False  # 有一个点可达，标识位设为 False
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)

            # 如果所有点都不可达，尝试脱困
            if all_unreachable:
                rospy.logwarn("[map_provider - auto_map - assigner.py] all unreachable, try to escape")
                escape_mode = True
                nearby_point = robots[ir].get_nearby_reachable_points()
                if nearby_point is not None:
                    rospy.loginfo(
                        "[map_provider - auto_map - assigner.py] Found nearby reachable point for robot %s: %s", ir,
                        nearby_point)
                    revenue_record.append(0)  # 脱困点的收益设为0
                    centroid_record.append(nearby_point)
                    id_record.append(ir)
                    rospy.loginfo("nearby_point is not None")
                    robots[ir].send_nav_goal(nearby_point, escape=True)
                    rospy.sleep(delay_after_assignement)
                else:
                    rospy.logwarn(
                        "[map_provider - auto_map - assigner.py] No nearby reachable point found for robot %s.", ir)
                break

        if len(na) < 1:
            revenue_record = []
            centroid_record = []
            id_record = []
            for ir in nb:
                for ip in range(0, len(centroids)):
                    cost = norm(robots[ir].get_position() - centroids[ip])
                    information_gain = infoGain[ip]
                    if (norm(robots[ir].get_position() - centroids[ip]) <= hysteresis_radius):
                        information_gain *= hysteresis_gain

                    if ((norm(centroids[ip] - robots[ir].assigned_point)) < hysteresis_radius):
                        information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]],
                                                           info_radius) * hysteresis_gain

                    revenue = information_gain * info_multiplier - cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)

        if len(revenue_record) < 1:
            # 取消所有目标
            rospy.logwarn("[map_provider - auto_map - assigner.py] No available revenue records, cancelling all goals.")
            for r in robots:
                r.cancel_nav_goal()
            rospy.sleep(1)
            # 向 (0, 0) 发送导航目标
            rospy.loginfo("[map_provider - auto_map - assigner.py] Navigating all robots to (0, 0).")
            for r in robots:
                r.send_nav_goal(array([0.0, 0.0]), exit=True)
            break

        target_flag = False
        if len(id_record) > 0:
            while len(revenue_record) > 0:
                winner_id = revenue_record.index(max(revenue_record))
                target_point = centroid_record[winner_id]
                tuple_target_point = (target_point[0], target_point[1])
                if tuple_target_point in ttl_set:
                    centroid_record.pop(winner_id)
                    revenue_record.pop(winner_id)
                    continue
                ttl_cnt = ttl_dict.get(tuple_target_point, 0) + 1
                ttl_dict[tuple_target_point] = ttl_cnt
                if ttl_cnt > 5:
                    rospy.logwarn("[map_provider - auto_map - assigner.py] point %s ttl -> 5, pop!", target_point)
                    ttl_set.add(tuple_target_point)
                    centroid_record.pop(winner_id)
                    revenue_record.pop(winner_id)
                else:
                    robots[id_record[winner_id]].send_nav_goal(target_point, escape=escape_mode)
                    rospy.sleep(delay_after_assignement)
                    escape_mode = False  # 重置标识位
                    target_flag = True
                    break
            if not target_flag:
                # 取消所有目标
                rospy.logwarn(
                    "[map_provider - auto_map - assigner.py] No available revenue records, cancelling all goals.")
                for r in robots:
                    r.cancel_nav_goal()
                rospy.sleep(1)
                # 向 (0, 0) 发送导航目标
                rospy.loginfo("[map_provider - auto_map - assigner.py] Navigating all robots to (0, 0).")
                for r in robots:
                    r.send_nav_goal(array([0.0, 0.0]), exit=True)
                break

        rate.sleep()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
