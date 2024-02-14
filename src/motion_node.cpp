#include <franka_sara_shield_controller/motion_node.h>

#include <ros/ros.h>

namespace franka_sara_shield_controller {

MotionNode::MotionNode():
  nh{},
  target_joint_pos_publisher_(nh.advertise<std_msgs::Float32MultiArray>("/sara_shield/goal_joint_pos", 100)),
  robot_current_pos_sub_(nh.subscribe("/sara_shield/observed_joint_pos", 100, &MotionNode::observeRobotJointCallback, this)),
  timer_(nh.createTimer(ros::Duration(0.05), &MotionNode::main_loop, this))
{
  joint_positions_.push_back(std::vector<float>{0.0, -0.78539, 0.0, -2.35619, 0.0, 1.5707, 0.78539});
  joint_positions_.push_back(std::vector<float>{1.0, -1.0, 0.5, -1.35619, 0.0, 1.5707, 0.78539});
  joint_positions_.push_back(std::vector<float>{-1.0, -0.4 , -0.5, -2.0 , 0.0, 2.3707, -0.78539});
}

void MotionNode::main_loop(const ros::TimerEvent &) {
  if(init_)
  {
    std_msgs::Float32MultiArray goal_joint_position;
    goal_joint_position.data = joint_positions_[this_motion_];
    target_joint_pos_publisher_.publish(goal_joint_position);
  }
}

void MotionNode::observeRobotJointCallback(const std_msgs::Float32MultiArray& msg) {
  if(!init_){  
    init_ = true;
  }
  std::vector<float> joint_position_vec(msg.data.begin(), msg.data.end());
  for (int i = 0; i < joint_position_vec.size(); i++) {
    if (std::abs(joint_position_vec[i] - joint_positions_[this_motion_][i]) > accuracy_) {
      return;
    }
  }
  // increase motion
  if (this_motion_ == joint_positions_.size() - 1) {
    this_motion_ = 0;
  } else {
    this_motion_++;
  }
}

}
