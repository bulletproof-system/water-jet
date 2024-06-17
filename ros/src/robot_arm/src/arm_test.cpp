#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include "robot_arm/AimAction.h"

#include <sound_play/SoundRequest.h>

#include <cmath>
#include "pot_database/GetPotInfo.h"
#include "pot_database/SetPotInfo.h"

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
    ROS_INFO("arm server starting......................................");
    as_.start();
    first_ = true;
    // center_point_sub_ = nh_.subscribe("object_center", 10, &ArmActionServer::objectsCenterCallback, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 30);
    mani_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl", 30);
    tts_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 20);
    // pot_get_client_ = nh_.serviceClient<pot_database::GetPotInfo>("/database/pot/get");
    // ros::service::waitForService("/database/pot/get");
    // pot_set_client_ = nh_.serviceClient<pot_database::SetPotInfo>("/database/pot/set");
    // ros::service::waitForService("/database/pot/set");

    // test
    target_point_.point.x = 1;
    target_point_.point.y = -1;
    target_point_.point.z = 2;
  }

  void objectsCenterCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    // unit test
    // if (!first_) return;
    // first_ = false;
    // target_point_ = *msg;
    return;
  }


  void executeCallback(const robot_arm::AimGoalConstPtr& goal)
  {
    ROS_INFO("Executing arm action.........................................");
  
    pot_database::GetPotInfo getPotInfo;
    getPotInfo.request.id = goal->id;

    adjust();
    extendArm();
    ros::Duration(3.0).sleep();
    speak();
    retractArm();

    robot_arm::AimResult result;
    result.success = true;
    result.info = "666";
    as_.setSucceeded(result);

    ROS_INFO("Aim action completed!.........................................");
  }

private:
  void speak() {
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.volume = 3.0;
    sp.arg = "watering";
    tts_pub_.publish(sp);
    ros::Duration(3.0).sleep();
  }

  void adjust()
  {
    ROS_INFO("begin moving action..........................");
    // here aim to the object pot...
    double target_x = target_point_.point.x;
    double target_y = target_point_.point.y;
    // double target_x = 1.0;
    // double target_y = 1.0;
    double angle = atan2(target_y, target_x);
    ROS_INFO("angle = %.2f ..................................", angle);
    geometry_msgs::Twist vel_cmd;

    // rotate for 1 second
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.angular.z = angle;
    vel_pub_.publish(vel_cmd);
    ros::Duration(1.0).sleep();

    // stop rotating
    vel_cmd.angular.z = 0;
    vel_pub_.publish(vel_cmd);
    ROS_INFO("stop moving action..........................");
    return;
  }

  void extendArm()
  {
    ROS_INFO("begin extending arm.......................................");
    sensor_msgs::JointState mani_ctrl_msg;
    mani_ctrl_msg.name.resize(2);
    mani_ctrl_msg.position.resize(2);
    mani_ctrl_msg.velocity.resize(2);
    mani_ctrl_msg.name[0] = "lift";
    mani_ctrl_msg.name[1] = "gripper";
    // lift the arm
    mani_ctrl_msg.position[0] = target_point_.point.z;
    // always hold on
    mani_ctrl_msg.position[1] = 0.032;
    mani_ctrl_msg.velocity[0] = 0.2;
    mani_ctrl_msg.velocity[1] = 5;
    mani_ctrl_pub_.publish(mani_ctrl_msg);
    ROS_INFO("finish extending arm.......................................");
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
    // put down the arm
    mani_ctrl_msg.position[0] = 0;
    // always hold on
    mani_ctrl_msg.position[1] = 0.032;
    mani_ctrl_msg.velocity[0] = 0.2;
    mani_ctrl_msg.velocity[1] = 5;
    mani_ctrl_pub_.publish(mani_ctrl_msg);
    ROS_INFO("finish retracting arm..................................");
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
