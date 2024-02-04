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
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

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

  bool new_goal_;
  int update_iteration_ = 0;
  std::vector<double> goal_joint_pos_;
  std::vector<std::vector<double>> human_meas_;

  ros::NodeHandle nh;
  ros::Publisher human_marker_pub_;
  ros::Subscriber robot_goal_pos_sub_;
  ros::Subscriber force_safe_sub_;
  ros::Subscriber force_unsafe_sub_;
  ros::Subscriber humans_in_scene_sub_;
  ros::Publisher robot_marker_pub_;
  ros::Publisher robot_current_pos_pub_;
  ros::Publisher sara_shield_safe_pub_;
  
  void createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, int color_type);
  void createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules);
  void createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);
  void createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);
  void visualizeRobotAndHuman();  
  
  // callbacks
  void goalJointPosCallback(const std_msgs::Float32MultiArray& msg);
  void forceSafeCallback(const std_msgs::Bool & msg);
  void forceUnsafeCallback(const std_msgs::Bool & msg);
  void humansInSceneCallback(const std_msgs::Bool& msg);

};

}  // namespace franka_sara_shield_controller
