#!/usr/bin/env python
import rospy
from pot_database.srv import DeletePot

def test_delete_pot():
    rospy.init_node('test_delete_pot')
    rospy.wait_for_service('/database/pot/delete')
    try:
        delete_pot = rospy.ServiceProxy('/database/pot/delete', DeletePot)
        response = delete_pot(id=1)
        print("Deleting Pot was successful: ", response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    test_delete_pot()
