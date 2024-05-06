#!/usr/bin/env python
import rospy
from database.srv import SetPotInfo, PotInfo

def test_set_pot_info():
    rospy.init_node('test_set_pot_info')
    rospy.wait_for_service('/database/pot/set')
    try:
        set_pot_info = rospy.ServiceProxy('/database/pot/set', SetPotInfo)
        pot_info = PotInfo(id=1, pose="pose_data", data="additional_data", picture="picture_data", active=True, last_water_date="2024-01-01")
        response = set_pot_info(pot_info)
        print("Setting Pot Info was successful: ", response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    test_set_pot_info()
