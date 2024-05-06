#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "robot_arm/AimAction.h"

#include <sound_play/SoundRequest.h>

#include <cmath>
#include "database/GetPotInfo.h"
#include "database/SetPotInfo.h"

#include <sstream>
#include <ctime>
#include <iomanip>

std::string getCurrentTime() {
    std::time_t now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y-%m-%dT%H:%M:%S");
    return ss.str();
}

class ArmActionServer
{
public:
  ArmActionServer(ros::NodeHandle& nh) : nh_(nh), as_(nh_, "/aim", boost::bind(&ArmActionServer::executeCallback, this, _1), false)
  {
    as_.start();
    first_ = true;
    center_point_sub_ = nh_.subscribe("object_center", 10, &ArmActionServer::objectsCenterCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    tts_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 20);
    pot_get_client_ = nh_.serviceClient<database::GetPotInfo>("/database/pot/get");
    ros::service::waitForService("/database/pot/get");
    pot_set_client_ = nh_.serviceClient<database::SetPotInfo>("/database/pot/set");
    ros::service::waitForService("/database/pot/set");

    // test
    // target_point_.point.x = 1.2;
    // target_point_.point.y = 4.0;
    // target_point_.point.z = 1.2;
  }

  void objectsCenterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    if (!first_) return;
    first_ = false;
    target_point_ = *msg;
    return;
  }


  void executeCallback(const robot_arm::AimGoalConstPtr& goal)
  {
    ROS_INFO("Executing arm action...");
  
    database::GetPotInfo getPotInfo;
    getPotInfo.request.id = goal->id;

    if (pot_get_client_.call(getPotInfo)) {
      if (getPotInfo.response.success) {
        ROS_INFO("succeed to get active infomation!");
        bool active = getPotInfo.response.info.active;
        if (active) {
          adjust();
          extendArm();
          ros::Duration(2.0).sleep();
          // simulate the process of jetting
          speak();
          retractArm();

          database::SetPotInfo setPotInfo;
          setPotInfo.request.info = getPotInfo.response.info;
          setPotInfo.request.info.last_water_date = getCurrentTime();
          if (pot_set_client_.call(setPotInfo)) {
            if (setPotInfo.response.success) {
              ROS_INFO("succeed to set pot's last water date!");
            }
            else {
              ROS_WARN("fail to set pot's last water date....");
            }
          }
          robot_arm::AimResult result;
          result.success = true;
          result.info = "the pot is active!";
          as_.setSucceeded(result);
        }
        else {
          robot_arm::AimResult result;
          result.success = true;
          result.info = "the pot is not active...";
          as_.setSucceeded(result);
        }
      }
    }
    else {
      ROS_WARN("fail to get pot infomation...");
      robot_arm::AimResult result;
      result.success = false;
      result.info = "fail to get pot infomation...";
      as_.setSucceeded(result);
    }

    ROS_INFO("Aim action completed!");
  }

private:
  void speak() {
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.volume = 3.0;
    sp.arg = "watering";
    tts_pub_.publish(sp);
  }

  void adjust()
  {
    ROS_INFO("begin moving action...");
    // here aim to the object pot...
    double target_x = target_point_.point.x;
    double target_y = target_point_.point.y;
    // double target_x = 1.0;
    // double target_y = 1.0;
    double angle = atan2(target_y, target_x);
    geometry_msgs::Twist vel_cmd;

    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.angular.z = angle;
    vel_pub_.publish(vel_cmd);
    ros::Duration(1.0).sleep();
    vel_cmd.angular.z = 0;
    vel_pub_.publish(vel_cmd);
    return;
  }

  void extendArm()
  {
    ROS_INFO("begin extending arm...");
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = target_point_.point.z;
    // always hold on
    mani_ctrl_msg.position[1] = 0.032;
    mani_ctrl_msg.velocity[0] = 0.2;
    mani_ctrl_msg.velocity[1] = 5;
    mani_ctrl_pub_.publish(mani_ctrl_msg);
    ROS_INFO("finish extending arm...");
  }

  void retractArm()
  {
    ROS_INFO("begin retracting arm...");
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    mani_ctrl_msg.position[0] = 0;
    // always hold on
    mani_ctrl_msg.position[1] = 0.032;
    mani_ctrl_msg.velocity[0] = 0.2;
    mani_ctrl_msg.velocity[1] = 5;
    mani_ctrl_pub_.publish(mani_ctrl_msg);
    ROS_INFO("finish retracting arm...");
  }

  bool first_;
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<robot_arm::AimAction> as_;
  ros::Subscriber center_point_sub_;
  ros::Publisher vel_pub_;
  ros::Publisher mani_ctrl_pub_;
  ros::Publisher tts_pub_;
  ros::ServiceClient pot_get_client_;
  ros::ServiceClient pot_set_client_;
  geometry_msgs::PointStamped target_point_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arm");
  ros::NodeHandle nh;

  ArmActionServer arm_action_server(nh);
  ros::spin();

  return 0;
}
