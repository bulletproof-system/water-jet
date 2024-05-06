#!/usr/bin/env python
import rospy
from database.srv import SetPotActive

def test_set_pot_active():
    rospy.init_node('test_set_pot_active')
    rospy.wait_for_service('/database/pot/set_active')
    try:
        set_pot_active = rospy.ServiceProxy('/database/pot/set_active', SetPotActive)
        response = set_pot_active(id=1, active=False)
        print("Setting Pot Active was successful: ", response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    test_set_pot_active()
