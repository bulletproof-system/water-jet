#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "robot_arm/AimAction.h"

typedef actionlib::SimpleActionClient<robot_arm::AimAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const robot_arm::AimResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %d", result->success);
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle nh;
  Client client("/aim", true);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();

  robot_arm::AimGoal goal;
  goal.id = 1;
  client.sendGoal(goal, &doneCb);

  ros::spin();

  return 0;
}
