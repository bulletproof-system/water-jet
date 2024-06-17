#!/usr/bin/env python
# coding=utf-8

#--------Include modules---------------
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from map_provider.msg import PointArray
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import Robot, informationGain, discount
from numpy.linalg import norm
from collections import defaultdict

# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()
frontiers=[]
global1=OccupancyGrid()
global2=OccupancyGrid()
global3=OccupancyGrid()
globalmaps=[]
def callBack(data):
	global frontiers
	frontiers=[]
	for point in data.points:
		frontiers.append(array([point.x,point.y]))

def mapCallBack(data):
    global mapData
    mapData=data
# Node----------------------------------------------

def node():
	global frontiers,mapData,global1,global2,global3,globalmaps
	rospy.init_node('assigner', anonymous=False)
	
	# fetching all parameters
	map_topic= rospy.get_param('~map_topic','/map')
	info_radius= rospy.get_param('~info_radius',1.0)					#this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
	info_multiplier=rospy.get_param('~info_multiplier',3.0)		
	hysteresis_radius=rospy.get_param('~hysteresis_radius',3.0)			#at least as much as the laser scanner range
	hysteresis_gain=rospy.get_param('~hysteresis_gain',2.0)				#bigger than 1 (biase robot to continue exploring current region
	frontiers_topic= rospy.get_param('~frontiers_topic','/filtered_points')	
	n_robots = rospy.get_param('~n_robots',1)
	namespace = rospy.get_param('~namespace','')
	namespace_init_count = rospy.get_param('namespace_init_count', 1)
	delay_after_assignement=rospy.get_param('~delay_after_assignement', 0.3)
	rateHz = rospy.get_param('~rate',100)
	
	rate = rospy.Rate(rateHz)
#-------------------------------------------
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
#---------------------------------------------------------------------------------------------------------------
		
# wait if no frontier is received yet 
	while len(frontiers)<1:
		pass
	centroids=copy(frontiers)	
#wait if map is not received yet
	while (len(mapData.data)<1):
		pass

	robots = []
	if len(namespace)>0:
		for i in range(0,n_robots):
			robots.append(Robot(namespace+str(i+namespace_init_count)))
	elif len(namespace)==0:
			robots.append(Robot(namespace))
	for i in range(0,n_robots):
		robots[i].sendGoal(robots[i].getPosition())
#-------------------------------------------------------------------------
#---------------------     Main   Loop     -------------------------------
#-------------------------------------------------------------------------
	# TTL dict & set
	ttl_dict = defaultdict()	# 用于统计目标点的访问次数
	ttl_set = set()				# 用于收纳 pop 掉的点

	while not rospy.is_shutdown():
		centroids = copy(frontiers)		
#-------------------------------------------------------------------------			
#Get information gain for each frontier point
		infoGain = []
		for ip in range(0, len(centroids)):
			infoGain.append(informationGain(mapData,[centroids[ip][0],centroids[ip][1]],info_radius))
#-------------------------------------------------------------------------			
#get number of available/busy robots
		na = [] #available robots
		nb = [] #busy robots
		for i in range(0, n_robots):
			if (robots[i].getState() == 1):
				nb.append(i)
			else:
				na.append(i)	
		# rospy.loginfo("available robots: " + str(na))
#------------------------------------------------------------------------- 
#get dicount and update informationGain
		for i in nb+na:
			infoGain=discount(mapData,robots[i].assigned_point,centroids,infoGain,info_radius)
#-------------------------------------------------------------------------            
		revenue_record = []
		centroid_record = []
		id_record = []
		
		escape_mode = False
		# 在 sendGoal 之前添加可达性检查
		for ir in na:
			all_unreachable = True  # 标识位，假设所有点都不可达
			for ip in range(0, len(centroids)):
				cost = norm(robots[ir].getPosition() - centroids[ip])       
				threshold = 1
				information_gain = infoGain[ip]
				if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
					information_gain *= hysteresis_gain
				revenue = information_gain * info_multiplier - cost
				
				# 检查目标是否可达
				if robots[ir].isReachable(robots[ir].getPosition(), centroids[ip]):
					rospy.loginfo("all_unreachable -> False")
					all_unreachable = False  # 有一个点可达，标识位设为 False
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
			
			# 如果所有点都不可达，尝试脱困
			if all_unreachable:
				rospy.logwarn("[map_provider - auto_map - assigner.py] all unreachable, try to escape")
				escape_mode = True
				nearby_point = robots[ir].getNearbyReachablePoint()
				if nearby_point is not None:
					rospy.loginfo("[map_provider - auto_map - assigner.py] Found nearby reachable point for robot %s: %s", ir, nearby_point)
					revenue_record.append(0)  # 脱困点的收益设为0
					centroid_record.append(nearby_point)
					id_record.append(ir)
					rospy.loginfo("nearby_point is not None")
					robots[ir].sendGoal(nearby_point, escape=True)
					rospy.sleep(delay_after_assignement)
				else:
					rospy.logwarn("[map_provider - auto_map - assigner.py] No nearby reachable point found for robot %s.", ir)
				break
		
		if len(na) < 1:
			revenue_record = []
			centroid_record = []
			id_record = []
			for ir in nb:
				for ip in range(0, len(centroids)):
					cost = norm(robots[ir].getPosition() - centroids[ip])		
					threshold = 1
					information_gain=infoGain[ip]
					if (norm(robots[ir].getPosition() - centroids[ip]) <= hysteresis_radius):
						information_gain *= hysteresis_gain
				
					if ((norm(centroids[ip]-robots[ir].assigned_point))<hysteresis_radius):
						information_gain = informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius) * hysteresis_gain

					revenue = information_gain * info_multiplier - cost
					revenue_record.append(revenue)
					centroid_record.append(centroids[ip])
					id_record.append(ir)
		
		if len(revenue_record) < 1:
			# 取消所有目标
			rospy.logwarn("[map_provider - auto_map - assigner.py] No available revenue records, cancelling all goals.")
			for r in robots:
				r.cancelGoal()
			rospy.sleep(1)
			# 向 (0, 0) 发送导航目标
			rospy.loginfo("[map_provider - auto_map - assigner.py] Navigating all robots to (0, 0).")
			for r in robots:
				r.sendGoal(array([0.0, 0.0]), exit=True)
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
					rospy.logwarn("point %s ttl -> 5, pop!", target_point)
					ttl_set.add(tuple_target_point)
					centroid_record.pop(winner_id)
					revenue_record.pop(winner_id)
				else:
					robots[id_record[winner_id]].sendGoal(target_point, escape=escape_mode)
					rospy.sleep(delay_after_assignement)
					escape_mode = False  # 重置标识位
					target_flag = True
					break
			if not target_flag:
				# 取消所有目标
				rospy.logwarn("[map_provider - auto_map - assigner.py] No available revenue records, cancelling all goals.")
				for r in robots:
					r.cancelGoal()
				rospy.sleep(1)
				# 向 (0, 0) 发送导航目标
				rospy.loginfo("[map_provider - auto_map - assigner.py] Navigating all robots to (0, 0).")
				for r in robots:
					r.sendGoal(array([0.0, 0.0]), exit=True)
				break
#------------------------------------------------------------------------- 
		rate.sleep()
#-------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
