#include <franka_sara_shield_controller/sara_shield_impedance_controller.h>

namespace franka_sara_shield_controller {

// SaraShieldImpedanceController::SaraShieldImpedanceController(){}

bool SaraShieldImpedanceController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  // Get initial params joint impedance controller.
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

  // Get initial params Cartesian impedance controller
  double translational_stiffness;
  if (!node_handle.getParam("translational_stiffness", translational_stiffness)) {
    ROS_ERROR("SaraShieldImpedanceController: Could not read parameter translational_stiffness");
    return false;
  }
  double rotational_stiffness;
  if (!node_handle.getParam("rotational_stiffness", rotational_stiffness)) {
    ROS_ERROR("SaraShieldImpedanceController: Could not read parameter rotational_stiffness");
    return false;
  }
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();

  double nullspace_stiffness;
  if (!node_handle.getParam("nullspace_stiffness", nullspace_stiffness)) {
    ROS_ERROR("SaraShieldImpedanceController: Could not read parameter nullspace_stiffness");
    return false;
  }
  nullspace_stiffness_target_ = nullspace_stiffness;

  // Get state interface
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "SaraShieldImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
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

  // Get joint effort interface
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

  // Set init values to zero.
  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  // Init dynamic reconfigure ROS node
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("/sara_shield/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(
        dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&SaraShieldImpedanceController::complianceParamCallback, this, _1, _2));

  // Init ROS subscribers
  desired_joint_state_sub_ = nh.subscribe("/sara_shield/desired_joint_state", 100, & SaraShieldImpedanceController::desiredJointStateCallback, this);
  impedance_mode_sub_ = nh.subscribe("/sara_shield/set_impedance_mode", 100, & SaraShieldImpedanceController::impedanceModeCallback, this);

  // Init ROS publisher
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  observed_joint_pos_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/sara_shield/observed_joint_pos", 100);

  return true;
}


void SaraShieldImpedanceController::starting(const ros::Time& time) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  q_d_.insert(q_d_.begin(), std::begin(robot_state.q), std::end(robot_state.q));
  initializeCartesianImpedanceMode();
}


void SaraShieldImpedanceController::initializeCartesianImpedanceMode() {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> q_d_arr;
  std::copy_n(q_d_.begin(), 7, q_d_arr.begin());
  std::array<double, 16> desired_transform_arr = model_handle_->getPose(
      franka::Frame::kEndEffector,
      q_d_arr,
      robot_state.F_T_EE,
      robot_state.EE_T_K);

  Eigen::Affine3d desired_transform(Eigen::Matrix4d::Map(desired_transform_arr.data()));

  // set equilibrium point to current state
  position_d_ = desired_transform.translation();
  orientation_d_ = Eigen::Quaterniond(desired_transform.rotation());
  position_d_target_ = desired_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(desired_transform.rotation());
}


void SaraShieldImpedanceController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  // Filter joint velocity
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - joint_vel_filter_) * dq_filtered_[i] + joint_vel_filter_ * robot_state.dq[i];
  }

  // Calculate the joint torques
  std::array<double, 7> tau_d_calculated;
  if (is_in_joint_impedance_state_) {
    tau_d_calculated = calculateTorquesJointImpedanceControl(robot_state.q, dq_filtered_, coriolis);
  } else {
    // get jacobian
    std::array<double, 42> jacobian =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    tau_d_calculated = calculateTorquesCartesianImpedanceControl(
      robot_state.q,
      dq_filtered_,
      robot_state.tau_J_d,
      robot_state.O_T_EE,
      coriolis,
      jacobian
    );
  }

  // Limit the joint difference
  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  // std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);
  std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, robot_state.tau_J_d);

  // Set the joint torque command
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_rate_limited[i]);
  }

  // Ramp up Impedance controller parameters to desired value.
  rampUpImpedanceParameters();

  // Publish the torque outputs for debugging
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
    last_tau_d_[i] = tau_d_calculated[i] + gravity[i];
  }

  // Publish the observed joint position
  std_msgs::Float32MultiArray pos_msg;
  std::vector<float> observed_pose_float(robot_state.q.begin(), robot_state.q.end());
  pos_msg.data = observed_pose_float;
  observed_joint_pos_pub_.publish(pos_msg);
}


std::array<double, 7> SaraShieldImpedanceController::calculateTorquesJointImpedanceControl(
    const std::array<double, 7>& joint_pos,
    const std::array<double, 7>& joint_vel,
    const std::array<double, 7>& coriolis
  ) {
  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (q_d_[i] - joint_pos[i]) +
                          d_gains_[i] * (dq_d_[i] - joint_vel[i]);
  }
  return tau_d_calculated;
}


std::array<double, 7> SaraShieldImpedanceController::calculateTorquesCartesianImpedanceControl(
    std::array<double, 7>& joint_pos,
    std::array<double, 7>& joint_vel,
    std::array<double, 7>& joint_torque,
    std::array<double, 16>& O_T_EE,
    std::array<double, 7>& coriolis,
    std::array<double, 42>& jacobian
  ) {
  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis_eigen(coriolis.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_eigen(jacobian.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_eigen(joint_pos.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_d_eigen(q_d_.data(), 7);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_eigen(joint_vel.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_eigen(joint_torque.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_example_controllers::pseudoInverse(jacobian_eigen.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian_eigen.transpose() *
              (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian_eigen * dq_eigen));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian_eigen.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_eigen - q_eigen) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq_eigen);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis_factor_ * coriolis_eigen;
  std::array<double, 7> final_torque;
  std::copy_n(tau_d.data(), 7, final_torque.begin());
  return final_torque;
}


void SaraShieldImpedanceController::rampUpImpedanceParameters() {
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
}


void SaraShieldImpedanceController::desiredJointStateCallback(const sensor_msgs::JointState& msg){
  q_d_ = msg.position;
  dq_d_ = msg.velocity;
  if (is_in_joint_impedance_state_ == false) {
    initializeCartesianImpedanceMode();
  }
}


void SaraShieldImpedanceController::impedanceModeCallback(const std_msgs::Bool& msg) {
  if (is_in_joint_impedance_state_ == msg.data) {
    return;
  }
  is_in_joint_impedance_state_ = msg.data;
  if (is_in_joint_impedance_state_ == false) {
    initializeCartesianImpedanceMode();
  }
}


void SaraShieldImpedanceController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}


std::array<double, 7> SaraShieldImpedanceController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, k_delta_tau_max_), -k_delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_sara_shield_controller

PLUGINLIB_EXPORT_CLASS(franka_sara_shield_controller::SaraShieldImpedanceController,
                       controller_interface::ControllerBase)
