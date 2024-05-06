#!/usr/bin/env python
import rospy
from pot_database.srv import GetPotList

def test_get_pot_list():
    rospy.init_node('test_get_pot_list')
    rospy.wait_for_service('/database/pot/list')
    try:
        get_pot_list = rospy.ServiceProxy('/database/pot/list', GetPotList)
        response = get_pot_list()
        print("List of Pots: ", response.pots)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    test_get_pot_list()
