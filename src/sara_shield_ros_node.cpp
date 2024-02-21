#include <franka_sara_shield_controller/sara_shield_ros_node.h>

#include <ros/ros.h>

namespace franka_sara_shield_controller {

SaraShieldRosNode::SaraShieldRosNode():
    nh{},
    robot_goal_pos_sub_(nh.subscribe("/sara_shield/goal_joint_pos", 100, & SaraShieldRosNode::goalJointPosCallback, this)),
    force_safe_sub_(nh.subscribe("/sara_shield/force_safe", 100, & SaraShieldRosNode::forceSafeCallback, this)),
    force_unsafe_sub_(nh.subscribe("/sara_shield/force_unsafe", 100, & SaraShieldRosNode::forceUnsafeCallback, this)),
    humans_in_scene_sub_(nh.subscribe("/sara_shield/humans_in_scene", 100, &SaraShieldRosNode::humansInSceneCallback, this)),
    robot_current_pos_sub_(nh.subscribe("/sara_shield/observed_joint_pos", 100, &SaraShieldRosNode::observeRobotJointCallback, this)),
    shield_mode_sub_(nh.subscribe("/sara_shield/shield_mode", 100, &SaraShieldRosNode::shieldModeCallback, this)),
    human_marker_pub_(nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/human_joint_marker_array", 100)),
    robot_marker_pub_(nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/robot_joint_marker_array", 100)),
    sara_shield_safe_pub_(nh.advertise<std_msgs::Bool>("/sara_shield/is_safe", 100)),
    desired_joint_state_pub_(nh.advertise<sensor_msgs::JointState>("/sara_shield/desired_joint_state", 100)),
    impedance_mode_pub_(nh.advertise<std_msgs::Bool>("/sara_shield/set_impedance_mode", 100)),
    timer_(nh.createTimer(ros::Duration(0.004), &SaraShieldRosNode::main_loop, this))
{
  ROS_WARN("Creating SaRA shield node.");
    // safety shield values
  double sample_time = 0.004;
  std::vector<double> init_qpos = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  std::string config_folder = std::getenv("SARA_SHIELD_CONFIG_PATH");
  std::string trajectory_config_file = config_folder + "/trajectory_parameters_panda.yaml";
  std::string robot_config_file = config_folder + "/robot_parameters_panda.yaml";
  std::string mocap_config_file = config_folder + "/asl_lab_mocap.yaml";
  ROS_WARN("Geting init params.");
  nh.getParam("/panda/init_x", init_x_);
  nh.getParam("/panda/init_y", init_y_);
  nh.getParam("/panda/init_z", init_z_);
  nh.getParam("/panda/init_roll", init_roll_);
  nh.getParam("/panda/init_pitch", init_pitch_);
  nh.getParam("/panda/init_yaw", init_yaw_);
  nh.getParam("/franka_control/arm_id", arm_id_);
  std::string type;
  nh.getParam("/sara_shield/shield_type", type);
  shield_type_ = getShieldTypeFromString(type);
  ROS_WARN_STREAM("Building SaRA shield with params: shield_type = " << type << ", " << 
    "init pos = [" <<
    std::to_string(init_x_) << ", " <<
    std::to_string(init_y_) << ", " <<
    std::to_string(init_z_) << "], init roll, pitch, yaw = [" <<
    std::to_string(init_roll_) << ", " <<
    std::to_string(init_pitch_) << ", " <<
    std::to_string(init_yaw_) << "]."
  );
  sendBaseTransform();
  sendImpedanceMode();
  // safety shield init
  // Right now, the roll and yaw rotation are confused in the safety shield. This will be fixed in the future.
  shield_ = new safety_shield::SafetyShield(
    sample_time,
    trajectory_config_file,
    robot_config_file,
    mocap_config_file,
    init_x_,
    init_y_,
    init_z_,
    init_yaw_,
    init_pitch_,
    init_roll_,
    init_qpos,
    shield_type_);
  ROS_WARN("Initializing SaRA shield subscribers.");
  // Human measurement
  int n_meas = sizeof(human_pose_sub_array_);
  human_meas_.resize(n_meas);
  human_pose_sub_array_[0] = nh.subscribe("/vrpn_client_node/human/head/pose", 240, & SaraShieldRosNode::humanPoseCallbackHead, this);
  human_pose_sub_array_[1] = nh.subscribe("/vrpn_client_node/human/clav/pose", 240, & SaraShieldRosNode::humanPoseCallbackClav, this);
  human_pose_sub_array_[2] = nh.subscribe("/vrpn_client_node/human/torso/pose", 240, & SaraShieldRosNode::humanPoseCallbackTorso, this);
  human_pose_sub_array_[3] = nh.subscribe("/vrpn_client_node/human/left_hand/pose", 240, & SaraShieldRosNode::humanPoseCallbackLeftHand, this);
  human_pose_sub_array_[4] = nh.subscribe("/vrpn_client_node/human/left_elbow/pose", 240, & SaraShieldRosNode::humanPoseCallbackLeftElbow, this);
  human_pose_sub_array_[5] = nh.subscribe("/vrpn_client_node/human/left_shoulder/pose", 240, & SaraShieldRosNode::humanPoseCallbackLeftShoulder, this);
  human_pose_sub_array_[6] = nh.subscribe("/vrpn_client_node/human/right_hand/pose", 240, & SaraShieldRosNode::humanPoseCallbackRightHand, this);
  human_pose_sub_array_[7] = nh.subscribe("/vrpn_client_node/human/right_elbow/pose", 240, & SaraShieldRosNode::humanPoseCallbackRightElbow, this);
  human_pose_sub_array_[8] = nh.subscribe("/vrpn_client_node/human/right_shoulder/pose", 240, & SaraShieldRosNode::humanPoseCallbackRightShoulder, this);
  
  for (int i=0; i<n_meas; i++) {
    human_meas_[i] = std::vector<double>{1200, 1200, 0};
  }
  last_human_meas_time_ = ros::Time::now();
  ROS_WARN("SaRA shield initialized.");
}

void SaraShieldRosNode::main_loop(const ros::TimerEvent &){
  if(!init_) {
    return;
  }
  shield_->humanMeasurement(human_meas_, last_human_meas_time_.toSec());

  // check if a new goal pose is set. If so, give a new LongTermTrajectory to sara shield
  if(new_goal_){
      new_goal_ = false;
      std::vector<double> qvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      shield_->newLongTermTrajectory(goal_joint_pos_, qvel);
      std::cout<<"new trajectory set"<<std::endl;
  }

  // Perform a sara shield update step
  safety_shield::Motion next_motion = shield_->step(ros::Time::now().toSec());
  // Send out the message
  sensor_msgs::JointState desired_joint_state_msg;
  desired_joint_state_msg.header.stamp = ros::Time::now();
  desired_joint_state_msg.position = next_motion.getAngle();
  desired_joint_state_msg.velocity = next_motion.getVelocity();
  desired_joint_state_pub_.publish(desired_joint_state_msg);

  /*if(!shield_->getSafety()){
      ROS_ERROR("NOT SAFE");
  }*/

  // visualize human every 100 iterations
  if(update_iteration_++ == visualize_every_){
      update_iteration_ = 0;
      sendBaseTransform();
      visualizeRobotAndHuman();
  }

  // Send status bool
  std_msgs::Bool status;
  status.data = shield_->getSafety();
  sara_shield_safe_pub_.publish(status);
}


void SaraShieldRosNode::resetShield() {
  shield_->reset(
    init_x_,
    init_y_,
    init_z_,
    init_yaw_,
    init_pitch_,
    init_roll_,
    current_joint_pos_,
    ros::Time::now().toSec(),
    shield_type_
  );
}


safety_shield::ShieldType SaraShieldRosNode::getShieldTypeFromString(std::string type) {
  if (type == "SSM") {
    return safety_shield::ShieldType::SSM;
  } else if (type == "PFL") {
    return safety_shield::ShieldType::PFL;
  } else if (type == "OFF") {
    return safety_shield::ShieldType::OFF;
  } else {
    ROS_ERROR_STREAM("Shield Type " << type << " unknown. Defaulting to SSM.");
    return safety_shield::ShieldType::SSM;
  }
}


void SaraShieldRosNode::sendImpedanceMode() {
  std_msgs::Bool status;
  if (shield_type_ == safety_shield::ShieldType::SSM || shield_type_ == safety_shield::ShieldType::PFL) {
    status.data = true;
  } else if (shield_type_ == safety_shield::ShieldType::OFF) {
    status.data = false;
  } else {
    ROS_ERROR_STREAM("shield_type_ unknown!!!");
    status.data = true;
  }
  impedance_mode_pub_.publish(status);
}

// ROS Callbacks

void SaraShieldRosNode::goalJointPosCallback(const std_msgs::Float32MultiArray& msg)
{
  std::vector<double> a(msg.data.begin(), msg.data.end());
  goal_joint_pos_ = a;
  new_goal_ = true;
}

void SaraShieldRosNode::observeRobotJointCallback(const std_msgs::Float32MultiArray& msg)
{
  current_joint_pos_ = std::vector<double>(msg.data.begin(), msg.data.end());
  if(!init_){
    // Right now, the roll and yaw rotation are confused in the safety shield. This will be fixed in the future.
    resetShield();
    init_ = true;
  }
}

void SaraShieldRosNode::forceSafeCallback(const std_msgs::Bool & msg){
  shield_->setForceSafe(msg.data);
}

void SaraShieldRosNode::forceUnsafeCallback(const std_msgs::Bool & msg){
  shield_->setForceUnsafe(msg.data);
}

//TODO: implement in sara_shield
void SaraShieldRosNode::humansInSceneCallback(const std_msgs::Bool& msg) {
  //if (!msg.data) {
  //  shield_->noHumanInTheScene();
  //}
}

void SaraShieldRosNode::shieldModeCallback(const std_msgs::String& msg) {
  shield_type_ = getShieldTypeFromString(msg.data);
  sendImpedanceMode();
  resetShield();
}

// VISUALIZATION
void SaraShieldRosNode::sendBaseTransform(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(init_x_, init_y_, init_z_) );
  tf::Quaternion q;
  q.setRPY(init_roll_, init_pitch_, init_yaw_);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", arm_id_ + base_id_));
}

void SaraShieldRosNode::visualizeRobotAndHuman(){
  // visualize the robot and human
  visualization_msgs::MarkerArray humanMarkerArray = visualization_msgs::MarkerArray();
  visualization_msgs::MarkerArray robotMarkerArray = visualization_msgs::MarkerArray();

  // visualization of robot and human capsules
  std::vector<std::vector<double>> humanCapsules = shield_->getHumanReachCapsules(0);
  createPoints(humanMarkerArray, 3 * humanCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 2);
  createCapsules(humanMarkerArray, humanCapsules);

  std::vector<std::vector<double>> robotReachCapsules = shield_->getRobotReachCapsules();
  
  createPoints(robotMarkerArray, 3 * robotReachCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 0);
  createCapsules(robotMarkerArray, robotReachCapsules);

  human_marker_pub_.publish(humanMarkerArray);
  robot_marker_pub_.publish(robotMarkerArray);
}

void SaraShieldRosNode::createPoints(visualization_msgs::MarkerArray& markers, int nb_points_to_add, int shape_type, 
    int color_type) {
  int prev_size = markers.markers.size();
  for(int i = 0; i < nb_points_to_add; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.ns = "shapes";
    marker.id = prev_size+i;
    marker.type = shape_type;
    if(color_type == 0) { // ROBOT
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
    }
    else if (color_type == 1) { // HUMAN_CYLINDER
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
    } else if (color_type == 2) { //HUMAN_REACH
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
    }   
    marker.color.a = 1.0f;
    marker.lifetime = ros::Duration();
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 0.0;
    markers.markers.push_back(marker);
  }
}

void SaraShieldRosNode::createCapsules(visualization_msgs::MarkerArray& markers, const std::vector<std::vector<double>>& capsules) { 
  auto marker = markers.markers.begin();
  for(const std::vector<double>& cap :capsules) {
    geometry_msgs::Point p1;
    p1.x = cap[0];
    p1.y = cap[1];
    p1.z = cap[2];
    geometry_msgs::Point p2;
    p2.x = cap[3];
    p2.y = cap[4];
    p2.z = cap[5];
    double radius = cap[6];
    //first circle
    createSphere(p1, radius, ros::Time::now(), *marker);
    marker++;
    //second circle
    createSphere(p2, radius, ros::Time::now(), *marker);
    marker++;
    //middle cylinder
    createCylinder(p1, p2, radius, ros::Time::now(), *marker);
    marker++;
  }
}

void SaraShieldRosNode::createSphere(const geometry_msgs::Point& pos, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.position = pos;
  marker.scale.x = 2*radius;
  marker.scale.y = 2*radius;
  marker.scale.z = 2*radius;
  marker.header.stamp = stamp;
}


void SaraShieldRosNode::createCylinder(const geometry_msgs::Point& p1, const geometry_msgs::Point p2, double radius, const ros::Time& stamp, visualization_msgs::Marker& marker) {
  double p1x = p1.x;
  double p1y = p1.y;
  double p1z = p1.z;
  double p2x = p2.x;
  double p2y = p2.y;
  double p2z = p2.z;
  double v2_x = (p2x-p1x);
  double v2_y = (p2y-p1y);
  double v2_z = (p2z-p1z);
  double norm = sqrt(pow(v2_x, 2) + pow(v2_y, 2) + pow(v2_z, 2));
  if(norm > 1e-6) {
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (p1x + p2x)/2;
    marker.pose.position.y = (p1y + p2y)/2;
    marker.pose.position.z = (p1z + p2z)/2;
    // Rotate z axis vector to direction vector according to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another/1171995#1171995
    double a_x = -v2_y/norm;
    double a_y = v2_x/norm;
    double a_z = 0;
    double a_w = 1 + v2_z/norm;
    double norm_q = sqrt(pow(a_w, 2) + pow(a_x, 2) + pow(a_y, 2) + pow(a_z, 2));
    marker.pose.orientation.w = a_w/norm_q;
    marker.pose.orientation.x = a_x/norm_q;
    marker.pose.orientation.y = a_y/norm_q;
    marker.pose.orientation.z = a_z/norm_q;
    marker.scale.z = norm;
    marker.scale.y = 2*radius;
    marker.scale.x = 2*radius;
  }
  else{
    marker.scale.x = 0;
    marker.scale.y = 0;
    marker.scale.z = 0;
  }
  marker.header.stamp = stamp;
}

void SaraShieldRosNode::humanPoseCallbackHead(const geometry_msgs::PoseStamped& msg) {
  human_meas_[0] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
  last_human_meas_time_ = ros::Time::now();
}
void SaraShieldRosNode::humanPoseCallbackClav(const geometry_msgs::PoseStamped& msg) {
  human_meas_[1] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackTorso(const geometry_msgs::PoseStamped& msg) {
  human_meas_[2] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackLeftHand(const geometry_msgs::PoseStamped& msg) {
  human_meas_[3] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackLeftElbow(const geometry_msgs::PoseStamped& msg) {
  human_meas_[4] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackLeftShoulder(const geometry_msgs::PoseStamped& msg) {
  human_meas_[5] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackRightHand(const geometry_msgs::PoseStamped& msg) {
  human_meas_[6] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackRightElbow(const geometry_msgs::PoseStamped& msg) {
  human_meas_[7] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}
void SaraShieldRosNode::humanPoseCallbackRightShoulder(const geometry_msgs::PoseStamped& msg) {
  human_meas_[8] = std::vector<double>{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
}

} // end namespace


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sara_shield_node");
    franka_sara_shield_controller::SaraShieldRosNode node;
    ros::spin();
    return 0;
}