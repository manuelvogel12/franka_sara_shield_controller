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
  // safety shield values
  bool activate_shield = true;
  double sample_time = 0.001;
  std::string folder = "/home/user/catkin_ws/src/franka_sara_shield_controller/config/";
  std::string trajectory_config_file = folder + "trajectory_parameters_panda.yaml";
  std::string robot_config_file = folder + "robot_parameters_panda.yaml";
  std::string mocap_config_file = folder + "cmu_mocap_no_hand.yaml";
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // safety shield init
  shield_ = new safety_shield::SafetyShield(
    sample_time,
    trajectory_config_file,
    robot_config_file,
    mocap_config_file,
    init_x,
    init_y,
    init_z,
    init_roll,
    init_pitch,
    init_yaw,
    init_qpos);

  // Dummy human measurement
  _human_meas.resize(21);
  for (int i=0; i<21; i++) {
    _human_meas[i] = std::vector<double>{1200, 1200, 0};
  }

  // ros publishers and subsribers
  robot_goal_pos_sub_ = nh.subscribe("/sara_shield/goal_joint_pos", 100, & SaraShieldController::goalJointPosCallback, this);

  return true;
}

void SaraShieldController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  // elapsed_time_ = ros::Duration(0.0);

  // reset sara shield
  std::vector<double> initial_pose_vec(initial_pose_.begin(), initial_pose_.end());
  shield_->reset(0, 0, 0, 0, 0, 0, initial_pose_vec, ros::Time::now().toSec());
}

void SaraShieldController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  // elapsed_time_ += period;

  shield_->humanMeasurement(_human_meas, ros::Time::now().toSec());

  // check if a new goal pose is set. If so, give a new LongTermTrajectory to sara shield
  if(_new_goal){
    _new_goal = false;
    std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    shield_->newLongTermTrajectory(_goal_joint_pos, qvel);
    std::cout<<"new trajectory set"<<std::endl;
  }

  // Perform a sara shield update step
  safety_shield::Motion next_motion = shield_->step(ros::Time::now().toSec());
  //std::cout<<"time"<<ros::Time::now().toSec()<<std::endl;
  std::vector<double> q = next_motion.getAngle(); 
  //for(double qi:q){
  //  std::cout<<qi<<" ";
  //}
  //std::cout<<std::endl;

  if(!shield_->getSafety()){
    ROS_ERROR("NOT SAFE");
  }

  // double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;
  for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(q[i]);
  }
}


// ROS Callbacks

void SaraShieldController::goalJointPosCallback(const std_msgs::Float32MultiArray& msg)
{
  std::vector<double> a(msg.data.begin(), msg.data.end());
  _goal_joint_pos = a;
  _new_goal = true;
}



}  // namespace franka_sara_shield_controller

PLUGINLIB_EXPORT_CLASS(franka_sara_shield_controller::SaraShieldController,
                       controller_interface::ControllerBase)
