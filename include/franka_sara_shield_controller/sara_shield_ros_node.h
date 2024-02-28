// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <cstdlib>

#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <franka_gripper/GraspActionGoal.h>
#include <franka_gripper/GraspActionResult.h>

#include "safety_shield/safety_shield.h"

namespace franka_sara_shield_controller {

class SaraShieldRosNode {
 public:
  SaraShieldRosNode();
  void main_loop(const ros::TimerEvent &);

 private:
  safety_shield::SafetyShield* shield_;
  safety_shield::ShieldType shield_type_= safety_shield::ShieldType::SSM;

  std::string arm_id_ = "panda";
  std::string base_id_ = "_link0";
  double init_x_, init_y_, init_z_, init_roll_, init_pitch_, init_yaw_;
  bool init_ = false;
  bool new_goal_ = false;
  int update_iteration_ = 0;
  int visualize_every_ = 10;
  ros::Time last_human_meas_time_;
  std::vector<double> goal_joint_pos_;
  std::vector<std::vector<double>> human_meas_;
  std::vector<double> current_joint_pos_;

  ros::NodeHandle nh;
  ros::Timer timer_;
  ros::Publisher human_marker_pub_;
  ros::Subscriber robot_goal_pos_sub_;
  ros::Subscriber force_safe_sub_;
  ros::Subscriber force_unsafe_sub_;
  ros::Subscriber humans_in_scene_sub_;
  ros::Subscriber robot_current_pos_sub_;
  ros::Subscriber shield_mode_sub_;
  ros::Subscriber gripper_command_sub_;
  ros::Subscriber gripper_success_sub_;
  // 9 measurements: head, clav, torso, left_hand, left_elbow, left_shoulder, right_hand, right_elbow, right_shoulder
  std::array<ros::Subscriber, 9> human_pose_sub_array_;
  ros::Publisher robot_marker_pub_;
  ros::Publisher sara_shield_safe_pub_;
  ros::Publisher desired_joint_state_pub_;
  ros::Publisher impedance_mode_pub_;
  ros::Publisher gripper_command_pub_;
  ros::Publisher gripper_success_pub_;
  
  void sendBaseTransform();
  void sendImpedanceMode();
  void resetShield();
  void createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, int color_type);
  void createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules);
  void createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);
  void createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker);
  void visualizeRobotAndHuman();  
  safety_shield::ShieldType getShieldTypeFromString(std::string type);
  
  // callbacks
  void goalJointPosCallback(const std_msgs::Float32MultiArray& msg);
  void observeRobotJointCallback(const std_msgs::Float32MultiArray& msg);
  void forceSafeCallback(const std_msgs::Bool & msg);
  void forceUnsafeCallback(const std_msgs::Bool & msg);
  void humansInSceneCallback(const std_msgs::Bool& msg);
  void shieldModeCallback(const std_msgs::String& msg);
  void gripperCommandCallback(const std_msgs::Bool& msg);
  void gripperSuccessCallback(const franka_gripper::GraspActionResult& msg);
  // human meas callbacks
  void humanPoseCallbackHead(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackClav(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackTorso(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackLeftHand(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackLeftElbow(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackLeftShoulder(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackRightHand(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackRightElbow(const geometry_msgs::PoseStamped& msg);
  void humanPoseCallbackRightShoulder(const geometry_msgs::PoseStamped& msg);
};

}  // namespace franka_sara_shield_controller
