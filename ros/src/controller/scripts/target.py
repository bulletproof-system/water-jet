# import rospy
# from std_msgs.msg import UInt8MultiArray, String
# from actionlib import SimpleActionServer
# from geometry_msgs.msg import Pose, Point, Quaternion
# from controller.msg import Hello, NodeInfo
# from controller.msg import TargetAction, TargetFeedback, TargetResult

# # TODO 包名
# from package.msg import NavigateAction, NavigateGoal
# from package.srv import CheckPot, CheckPotRequest

# class TargetNode:
#     def __init__(self):
#         rospy.init_node('target')
        
#         # Subscribers
#         self.hello_subscriber = rospy.Subscriber('/ctrl/hello', Hello, self.hello_callback)
        
#         # Publishers
#         self.node_info_publisher = rospy.Publisher('/ctrl/node_info', NodeInfo, queue_size=10)
        
#         # Action Server
#         self.server = SimpleActionServer('/controller/action/Target', TargetAction, execute_cb=self.execute_cb, auto_start=False)
#         self.server.start()

#     def hello_callback(self, msg):
#         rospy.loginfo("Received hello message")
#         node_info = NodeInfo()
#         node_info.status = "active"
#         self.node_info_publisher.publish(node_info)

#     def get_target_pose(self, target_id):
#         # fetch the coordinates from the database
#         return Pose(Point(x=1.0, y=2.0, z=0.0), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    
#     def navigate_to_target(self, pose):
#         goal = NavigateGoal()
#         goal.pos = pose
#         self.nav_client.send_goal(goal)
#         self.nav_client.wait_for_result()
#         nav_result = self.nav_client.get_result()

#         if nav_result.result == 'success':
#             return True
#         else:
#             rospy.logwarn("Navigation failed with result: %s" % nav_result.result)
#             return False

#     def check_flowerpot(self, pot_id):
#         """Check the presence of a flowerpot using the object_detect/check_pot service."""
#         try:
#             request = CheckPotRequest(id=pot_id)
#             response = self.check_pot_service(request)
#             return response.success
#         except rospy.ServiceException as e:
#             rospy.logwarn("Service call failed: %s" % e)
#             return False
        
#     def execute_cb(self, goal):
#         feedback = TargetFeedback()
#         result = TargetResult()
        
#         rospy.loginfo("Starting watering process for targets: %s" % goal.targets)
#         for i, target in enumerate(goal.targets):
#             #* 调用导航模块
#             target_pose = self.get_target_pose(target)
#             success = self.navigate_to_target(target_pose)
#             if not success:
#                 result.result = 'fail'
#                 self.server.set_aborted(result)
#                 return

#             #* 调用花盆识别模块
#             if not self.check_flowerpot(target):
#                 rospy.logwarn("No flowerpot detected at target: %d" % target)
#                 result.result = 'fail'
#                 self.server.set_aborted(result)
#                 return

#             # TODO 调用浇水jet模块

#             feedback.percentage = int((i + 1) * 100.0 / len(goal.targets))
#             feedback.target = target
#             self.server.publish_feedback(feedback)
#             rospy.loginfo("Watering at target %d" % target)
#             if self.server.is_preempt_requested():
#                 rospy.loginfo("Watering preempted")
#                 self.server.set_preempted()
#                 result.result = 'cancel'
#                 return
        
#         rospy.loginfo("Watering complete")
#         result.result = 'success'
#         self.server.set_succeeded(result)

# if __name__ == '__main__':
#     try:
#         target_node = TargetNode()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
