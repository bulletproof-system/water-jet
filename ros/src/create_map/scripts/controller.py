#!/usr/bin/env python
# coding=utf-8

import os
import rospy
from . import Base
import datetime
import multiprocessing

params = {}

class Controller:
    def __init__(self):

        def target():
            # TO BE DONE
            if params['simulate']:
                os.system("roslaunch <my_pkg> sim_init.launch")
            else:
                os.system("roslaunch <my_pkg> real_init.launch")

        process = multiprocessing.Process(target=target)
        process.start()
        self.process = process

        rospy.init_node("controller")
        
        for key in params:
            rospy.set_param(key, params[key])      

    def create_map_start(self):
        """
        Controller控制create_map开始建图

        Args: 
            None
        """
        client = rospy.ServiceProxy('/control/create_map/start', Base)
        rospy.wait_for_service('/control/create_map/start')
        resp = client('start')
        
        rospy.loginfo(resp.response)


    def create_map_save(self, map_name=None):
        """
        Controller控制create_map进行地图保存

        Args:
            map_name (str) : 图片名字
        """
        client = rospy.ServiceProxy('/control/create_map/save', Base)
        rospy.wait_for_service('/control/create_map/save')

        if map_name is None:
            # Default Map Name
            map_name = datetime.datetime.now().strftime("map_%Y%m%d_%H%M%S")

        path = params['pkg_path'] + '/maps/'  + map_name
        resp = client(path)
        rospy.loginfo(resp.response)



