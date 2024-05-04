#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "robot_arm/AimAction.h"

typedef actionlib::SimpleActionClient<robot_arm::AimAction> Client;

// 当目标完成时，会调用这个回调函数
void doneCb(const actionlib::SimpleClientGoalState& state,
            const robot_arm::AimResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Result: %d", result->success); // 输出结果
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle nh;
  Client client("aim", true); // 定义客户端，并指向服务器的名称 "aim"

  // 等待服务器启动
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();

  // 创建并发送目标
  robot_arm::AimGoal goal;
  goal.id = 1; // 假设的 pot_id 你可以根据需要修改这个值
  client.sendGoal(goal, &doneCb);

  ros::spin();

  return 0;
}