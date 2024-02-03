// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float32MultiArray.h>

#include "safety_shield/safety_shield.h"

namespace franka_sara_shield_controller {

class SaraShieldController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  // SaraShieldController();
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  // ros::Duration elapsed_time_;

  //todo remove and only use local vector
  std::array<double, 7> initial_pose_{};
  safety_shield::SafetyShield* shield_;

  bool _new_goal;
  std::vector<double> _goal_joint_pos;
  std::vector<std::vector<double>> _human_meas;

  ros::NodeHandle nh;
  ros::Subscriber robot_goal_pos_sub_;

  void goalJointPosCallback(const std_msgs::Float32MultiArray& msg);
};

}  // namespace franka_sara_shield_controller
