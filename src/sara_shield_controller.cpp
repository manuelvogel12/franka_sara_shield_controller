#include <franka_sara_shield_controller/sara_shield_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_sara_shield_controller {

// SaraShieldController::SaraShieldController(){}

bool SaraShieldController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR("SaraShieldController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("SaraShieldController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("SaraShieldController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "SaraShieldController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "SaraShieldController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_sara_shield_controller move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }
  joint_pos_sub_ = nh.subscribe("/sara_shield/joint_pos_output", 100, & SaraShieldController::jointPosCallback, this);
  observed_joint_pos_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/sara_shield/current_joint_pos", 100);

  return true;
}


void SaraShieldController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    q_[i] = position_joint_handles_[i].getPosition();
  }
  // elapsed_time_ = ros::Duration(0.0);
}


void SaraShieldController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  // elapsed_time_ += period;

  // set joint pos
  for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(q_[i]);
  }

  // observe joint pose
  std::vector<double> observed_pose(7, 0.0);
  for (size_t i = 0; i < 7; ++i) {
    observed_pose[i] = position_joint_handles_[i].getPosition();
  }
  std_msgs::Float32MultiArray pos_msg;
  std::vector<float> observed_pose_float(observed_pose.begin(), observed_pose.end());
  pos_msg.data = observed_pose_float;
  observed_joint_pos_pub_.publish(pos_msg);

}

void SaraShieldController::jointPosCallback(const std_msgs::Float64MultiArray& msg){
  q_ = msg.data;
}



}  // namespace franka_sara_shield_controller

PLUGINLIB_EXPORT_CLASS(franka_sara_shield_controller::SaraShieldController,
                       controller_interface::ControllerBase)
