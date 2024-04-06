#!/usr/bin/env python
# coding=utf-8

import rospy
import multiprocessing
import os
from create_map.srv import Base, BaseResponse

class MapCreator:
    def __init__(self):
        rospy.init_node("create_map")
        rospy.loginfo("map creator start!")

        rospy.Service('/control/create_map/start', Base, self.start)
        rospy.Service('/control/create_map/save', Base, self.save)

        self.process = None
    
    def start(self, req):
        if self.process != None:
            return BaseResponse("Map Creator is Running")
        
        def target():
            os.system("roslaunch create_map create_map.launch")
            # if rospy.get_param("simulate"):
            #     os.system("roslaunch create_map create_map_sim.launch")
            # else:
            #     # TO BE DONE
            #     os.system("roslaunch create_map create_map_real.launch")
        
        self.process = multiprocessing.Process(target=target)
        self.process.start()        

        return BaseResponse("map creating success")
    
    def save(self, req):
        map_name = req.request
        os.system('rosrun map_server map_saver -f ' + map_name)

        if self.process is not None:
            self.process.terminate()
            self.process = None
        
        return BaseResponse("map saving success")
    

MapCreator()
rospy.spin()

if __name__ == '__main__':
    MapCreator()
    rospy.spin()