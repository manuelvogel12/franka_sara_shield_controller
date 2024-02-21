/* 
This is an impedance controller that follows SaRA shield outputs.
It has two modes:
  - Joint impedance control to follow SaRA shield
  - Cartesian impedance control to hold a static position

Author: Jakob Thumm
Date: 20.02.2024
*/
#pragma once

// std
#include <memory>
#include <array>
#include <string>
#include <vector>
#include <iterator>
#include <mutex>
#include <cmath>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/node_handle.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.h>

// ROS messages
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

// Franka ROS
#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka/robot_state.h>
#include <franka/rate_limiting.h>
#include <franka/model.h>


namespace franka_sara_shield_controller {

class SaraShieldImpedanceController : public controller_interface::MultiInterfaceController<
                                           franka_hw::FrankaModelInterface,
                                           hardware_interface::EffortJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  // <--------- PUBLIC Functions ----------->
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Sends out the desired joint torque
  std::vector<hardware_interface::JointHandle> joint_handles_;
  // Get the current robot state
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  // Get the current robot model
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  /** 
   * True if the controller is in the joint impedance mode.
   * False if the controller is in the Cartesian impedance mode.
  */
  bool is_in_joint_impedance_state_ = true;

  // Limits the change in torque. Currently not used.
  static constexpr double k_delta_tau_max_{1.0};
  // Factor for the joint velocity filter. 1 = measured vel, 0 = previous vel.
  static constexpr double joint_vel_filter_{0.99};
  // <------- Joint impedance controller parameters --------->

  // Stiffness of the joint impedance controller
  std::vector<double> k_gains_;
  // Damping of the joint impedance controller
  std::vector<double> d_gains_;
  // Coriolis factor for the joint impedance controller. 1 = full, 0 = no coriolis force.
  double coriolis_factor_{1.0};

  // <------- Cartesian impedance controller parameters --------->

  /* This is used to slowly ramp up the current position/orientation/stiffness/damping 
   * parameters of the joint the Cartesian controller to the desired parameters
   * after changing the parameters.
   */
  double filter_params_{0.005};
  /* Nullspace stiffness of the Cartesian controller.
   * This defines the stiffness towards joint changes.
   */ 
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  // Stiffness of the Cartesian impedance controller
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  // Damping of the Cartesian impedance controller
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  
  // Desired EEF position
  Eigen::Vector3d position_d_;
  Eigen::Vector3d position_d_target_;
  // Desired EEF orientation
  Eigen::Quaterniond orientation_d_;
  Eigen::Quaterniond orientation_d_target_;
  // ???
  std::mutex position_and_orientation_d_target_mutex_;
  
  
  // Filtered joint velocity
  std::array<double, 7> dq_filtered_;
  // Init robot pose
  std::array<double, 16> initial_pose_;
  // Desired joint position
  std::vector<double> q_d_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  // Desired joint velocity.
  std::vector<double> dq_d_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Trigger for outputting controller information (mainly for debugging purposes)
  franka_hw::TriggerRate rate_trigger_{1.0};
  // Previous desired torque for calculating the torque error.
  std::array<double, 7> last_tau_d_{};
  // Publisher that outputs the controller information (mainly for debugging purposes)
  realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison> torques_publisher_;

  // The node handle
  ros::NodeHandle nh;
  // The node handle for dynamically setting the Cartesian impedance control parameters
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;

  // <------- ROS Subscribers --------->
  // Subscribes to the desired joint state that the robot should move to.
  ros::Subscriber desired_joint_state_sub_;
  // Subscriber that changes the control mode from joint impedance (true) to Cartesian impedance (false)
  ros::Subscriber impedance_mode_sub_;

  // <------- ROS Publishers --------->
  // Publishes the current joint position.
  ros::Publisher observed_joint_pos_pub_;
  
  // <--------- PRIVATE Functions ----------->
  /** Initialize the cartesian impedance controller.
  */
  void initializeCartesianImpedanceMode();

  /** Calculate the joint torques when in joint impedance control mode.
   * @param[in] joint_pos: current joint position
   * @param[in] joint_vel: current joint velocity (often the filtered one)
   * @param[in] coriolis: current coriolis force
   * @returns desired joint torques
   */
  std::array<double, 7> calculateTorquesJointImpedanceControl(
    const std::array<double, 7>& joint_pos,
    const std::array<double, 7>& joint_vel,
    const std::array<double, 7>& coriolis
  );

  /** Calculate the joint torques when in Cartesian impedance control mode.
   * @param[in] joint_pos: current joint position
   * @param[in] joint_vel: current joint velocity (often the filtered one)
   * @param[in] joint_torque: current joint torque from robot state
   * @param[in] O_T_EE: current transformation matrix from origin to end-effector
   * @param[in] coriolis: current coriolis force
   * @param[in] jacobian: current Jacobian matrix
   * @returns desired joint torques
   */
  std::array<double, 7> calculateTorquesCartesianImpedanceControl(
    std::array<double, 7>& joint_pos,
    std::array<double, 7>& joint_vel,
    std::array<double, 7>& joint_torque,
    std::array<double, 16>& O_T_EE,
    std::array<double, 7>& coriolis,
    std::array<double, 42>& jacobian
  );

  // Brings the Cartesian impedance control parameters closer to their desired target. 
  void rampUpImpedanceParameters();

  /** Calculate the forward kinematics for the given joint configuration.
   * @param[in] q: joint positions
   * @returns 4x4 pose matrix for the given frame in base frame.
   */
  std::array<double, 16> forwardKinematics(const std::array<double, 7>& q);

  // callbacks
  /** Set the desired joint position from the state.
   * @param[in] msg: JointState message
   */
  void desiredJointStateCallback(const sensor_msgs::JointState& msg);

  /** Set the impedance controller mode.
   * @details if msg=False, i.e., mode = Cartesian, we call initialize_cartesian_impedance_mode().
   *
   * @param[in] msg: Controller mode. 
   *                 True = joint impedance controller. 
   *                 False = Cartesian impedance controller.
   */
  void impedanceModeCallback(const std_msgs::Bool& msg);

  /** Set the parameters of the Cartesian impedance controller.
   * @param[in] config: Cartesian impedance controller parameters
   * @param[in] level: ???
   */
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);


  // <--------- Currently unused functions ----------->
  /** Calculate the torque saturation.
   * @details We currently use franka::limitRate() instead of this function!
   * @details This function makes sure the torque is not changing too fast.
   *          Limits the change in torque to k_delta_tau_max_.
   * @param[in] tau_d_calculated desired torque
   * @param[in] tau_J_d current torque taken from the robot state
   * @returns tau_d_saturated: saturated torque to execute.
  */
  std::array<double, 7> saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)
};

}  // namespace franka_sara_shield_controller
