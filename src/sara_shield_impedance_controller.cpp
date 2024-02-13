#include <franka_sara_shield_controller/sara_shield_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

namespace franka_sara_shield_controller {

// SaraShieldImpedanceController::SaraShieldImpedanceController(){}

bool SaraShieldImpedanceController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("SaraShieldImpedanceController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("SaraShieldImpedanceController: Could not parse joint names");
  }

  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("SaraShieldImpedanceController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "SaraShieldImpedanceController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "SaraShieldImpedanceController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("SaraShieldImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("SaraShieldImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  // Get state interface
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_state"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // Get model interface
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "SaraShieldImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  joint_pos_sub_ = nh.subscribe("/sara_shield/joint_pos_output", 100, & SaraShieldImpedanceController::jointPosCallback, this);
  observed_joint_pos_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/sara_shield/current_joint_pos", 100);

  return true;
}


void SaraShieldImpedanceController::starting(const ros::Time& /* time */) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_.insert(q_.begin(), std::begin(robot_state.q), std::end(robot_state.q));
}


void SaraShieldImpedanceController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (q_[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }

  // observe joint pose
  std_msgs::Float32MultiArray pos_msg;
  std::vector<float> observed_pose_float(robot_state.q.begin(), robot_state.q.end());
  pos_msg.data = observed_pose_float;
  observed_joint_pos_pub_.publish(pos_msg);
}

void SaraShieldImpedanceController::jointPosCallback(const std_msgs::Float64MultiArray& msg){
  q_ = msg.data;
}

std::array<double, 7> SaraShieldImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_sara_shield_controller

PLUGINLIB_EXPORT_CLASS(franka_sara_shield_controller::SaraShieldImpedanceController,
                       controller_interface::ControllerBase)
