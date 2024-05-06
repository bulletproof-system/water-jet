#!/usr/bin/env python
import rospy
from pot_database.srv import GetPotInfo

def test_get_pot_info():
    rospy.init_node('test_get_pot_info')
    rospy.wait_for_service('/database/pot/get')
    try:
        get_pot_info = rospy.ServiceProxy('/database/pot/get', GetPotInfo)
        response = get_pot_info(id=1)
        if response.success:
            print("Pot Info: ", response.info)
        else:
            print("Failed to get pot info.")
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    test_get_pot_info()
