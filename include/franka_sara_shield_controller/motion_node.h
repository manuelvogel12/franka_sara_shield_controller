// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <cstdlib>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

namespace franka_sara_shield_controller {

class MotionNode {
 public:
  MotionNode();
  void main_loop(const ros::TimerEvent &);

 private:
  std::vector<std::vector<float>> joint_positions_;
  int this_motion_ = 0;
  float accuracy_ = 0.05;
  bool init_ = false;

  ros::NodeHandle nh;
  ros::Timer timer_;
  ros::Publisher target_joint_pos_publisher_;
  ros::Subscriber robot_current_pos_sub_;

  void observeRobotJointCallback(const std_msgs::Float32MultiArray& msg);
};

}  // namespace franka_sara_shield_controller

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "motion_node");
    franka_sara_shield_controller::MotionNode node;
    ros::spin();
    return 0;
}