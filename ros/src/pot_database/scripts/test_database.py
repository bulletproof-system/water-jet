#!/usr/bin/env python
# coding=utf-8

import unittest
import rospy
from pot_database.srv import GetPotList, SetPotInfo, SetPotActive, GetPotInfo, DeletePot
from pot_database.msg import PotInfo
import pickle
from datetime import datetime
from geometry_msgs.msg import Pose

class TestDatabase(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_database')

        self.list_client = rospy.ServiceProxy('/database/pot/list', GetPotList)
        self.set_client = rospy.ServiceProxy('/database/pot/set', SetPotInfo)
        self.set_active_client = rospy.ServiceProxy('/database/pot/set_active', SetPotActive)
        self.get_client = rospy.ServiceProxy('/database/pot/get', GetPotInfo)
        self.delete_client = rospy.ServiceProxy('/database/pot/delete', DeletePot)

        rospy.sleep(2) # Give some time for services to become available

    def test_list_pots(self):
        # 创建一个示例的 Pose 对象
        sample_pose = Pose()
        sample_pose.position.x = 1.0
        sample_pose.position.y = 2.0
        sample_pose.position.z = 3.0
        sample_pose.orientation.x = 0.0
        sample_pose.orientation.y = 0.0
        sample_pose.orientation.z = 0.0
        sample_pose.orientation.w = 1.0
        # Creating a sample PotInfo
        pot_info = PotInfo(
        id=1,
        pose=sample_pose,
        data=pickle.dumps("sample_data"),
        picture=pickle.dumps("sample_picture"),
        active=True,
        last_water_date= ""
        )
        set_response = self.set_client(pot_info)
        self.assertTrue(set_response.success)

        response = self.list_client()
        self.assertTrue(len(response.pots) >= 1) # Adjust assertion based on expected conditions

    def test_set_and_get_pot_info(self):
        # 创建一个示例的 Pose 对象
        sample_pose = Pose()
        sample_pose.position.x = 1.0
        sample_pose.position.y = 2.0
        sample_pose.position.z = 3.0
        sample_pose.orientation.x = 0.0
        sample_pose.orientation.y = 0.0
        sample_pose.orientation.z = 0.0
        sample_pose.orientation.w = 1.0
        # Creating a sample PotInfo
        pot_info = PotInfo(
        id=1,
        pose=sample_pose,
        data=pickle.dumps("sample_data"),
        picture=pickle.dumps("sample_picture"),
        active=True,
        last_water_date= ""
        )
        set_response = self.set_client(pot_info)
        self.assertTrue(set_response.success)

        get_response = self.get_client(1)
        self.assertTrue(get_response.success)
        self.assertEqual(get_response.info.id, 1)
        self.assertTrue(get_response.info.active)

    def test_set_active(self):
        active_response = self.set_active_client(True, 1)
        self.assertTrue(active_response.success)

    def test_delete_pot(self):
        # 创建两个 PotInfo 对象并设置
        # 创建一个示例的 Pose 对象
        sample_pose1 = Pose()
        sample_pose1.position.x = 1.0
        sample_pose1.position.y = 2.0
        sample_pose1.position.z = 3.0
        sample_pose1.orientation.x = 0.0
        sample_pose1.orientation.y = 0.0
        sample_pose1.orientation.z = 0.0
        sample_pose1.orientation.w = 1.0

        # 创建一个示例的 Pose 对象
        sample_pose2 = Pose()
        sample_pose2.position.x = 3.0
        sample_pose2.position.y = 2.0
        sample_pose2.position.z = 1.0
        sample_pose2.orientation.x = 0.0
        sample_pose2.orientation.y = 0.0
        sample_pose2.orientation.z = 0.0
        sample_pose2.orientation.w = 1.0

        pot_info1 = PotInfo(
        id=1,
        pose=sample_pose1,
        data=pickle.dumps("pot_data1"),
        picture=pickle.dumps("picture_data1"),
        active=True,
        last_water_date=""
        )
        pot_info2 = PotInfo(
        id=2,
        pose=sample_pose2,
        data=pickle.dumps("pot_data2"),
        picture=pickle.dumps("picture_data2"),
        active=True,
        last_water_date=""
        )

        # 设置 pot 信息
        self.set_client(pot_info1)
        self.set_client(pot_info2)

        # 删除编号为 2 的 pot
        delete_response = self.delete_client(2)
        self.assertTrue(delete_response.success, "Failed to delete pot with id 2")

        # 获取所有 pots 列表
        list_response = self.list_client()
        self.assertTrue(list_response.pots, "No pots found after delete operation")

        # 检查是否只有编号为 1 的 pot 存在
        pots_ids = [pot.id for pot in list_response.pots]
        self.assertIn(1, pots_ids, "Pot with id 1 is missing in the list")
        self.assertNotIn(2, pots_ids, "Pot with id 2 was not deleted properly")


if __name__ == '__main__':
    import rostest
    rostest.rosrun("pot_database", "test_database", TestDatabase)