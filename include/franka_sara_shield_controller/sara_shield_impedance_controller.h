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
#include <std_msgs/Float64MultiArray.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>

namespace franka_sara_shield_controller {

class SaraShieldImpedanceController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaModelInterface,
                                           hardware_interface::PositionJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  // SaraShieldController();
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  // ros::Duration elapsed_time_;

  std::vector<double> q_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  ros::NodeHandle nh;
  ros::Subscriber joint_pos_sub_;
  ros::Publisher observed_joint_pos_pub_;
  
  // callbacks
  void jointPosCallback(const std_msgs::Float64MultiArray& msg);
};

}  // namespace franka_sara_shield_controller
