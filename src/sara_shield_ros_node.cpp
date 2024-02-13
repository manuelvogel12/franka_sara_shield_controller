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
    human_marker_pub_(nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/human_joint_marker_array", 100)),
    robot_marker_pub_(nh.advertise<visualization_msgs::MarkerArray>("/sara_shield/robot_joint_marker_array", 100)),
    sara_shield_safe_pub_(nh.advertise<std_msgs::Bool>("/sara_shield/is_safe", 100)),
    desired_joint_state_pub_(nh.advertise<sensor_msgs::JointState>("/sara_shield/desired_joint_state", 100)),
    timer_(nh.createTimer(ros::Duration(0.004), &SaraShieldRosNode::main_loop, this))
{
    // safety shield values
  double sample_time = 0.004;
  // TODO: MAKE DYNAMIC!
  std::string config_folder = std::getenv("SARA_SHIELD_CONFIG_PATH");
  std::string trajectory_config_file = config_folder + "/trajectory_parameters_panda.yaml";
  std::string robot_config_file = config_folder + "/robot_parameters_panda.yaml";
  std::string mocap_config_file = config_folder + "/cmu_mocap_no_hand.yaml";
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_roll = 0.0;
  double init_pitch = 0.0;
  double init_yaw = 0.0;
  std::vector<double> init_qpos = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};

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
  human_meas_.resize(21);
  for (int i=0; i<21; i++) {
    human_meas_[i] = std::vector<double>{1200, 1200, 0};
  }

}

void SaraShieldRosNode::main_loop(const ros::TimerEvent &){

    if(init_)
    {
        shield_->humanMeasurement(human_meas_, ros::Time::now().toSec());

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

        if(!shield_->getSafety()){
            ROS_ERROR("NOT SAFE");
        }

        // visualize human every 100 iterations
        if(update_iteration_++ == 10){
            update_iteration_ = 0;
            visualizeRobotAndHuman();
        }

        // Send status bool
        std_msgs::Bool status;
        status.data = shield_->getSafety();
        sara_shield_safe_pub_.publish(status);
    }
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
  if(!init_){
    std::vector<double> initial_pose_vec(msg.data.begin(), msg.data.end());
    shield_->reset(0, 0, 0, 0, 0, 0, initial_pose_vec, ros::Time::now().toSec());
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

// VISUALIZATION

void SaraShieldRosNode::visualizeRobotAndHuman(){
  // visualize the robot and human
  visualization_msgs::MarkerArray humanMarkerArray = visualization_msgs::MarkerArray();
  visualization_msgs::MarkerArray robotMarkerArray = visualization_msgs::MarkerArray();

  // visualization of robot and human capsules
  /* TODO: fix
  std::vector<std::vector<double>> humanCapsules = shield_->getHumanReachCapsules(1);
  createPoints(humanMarkerArray, 3 * humanCapsules.size(),
               visualization_msgs::Marker::CYLINDER, 2);
  createCapsules(humanMarkerArray, humanCapsules);
  */

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
    marker.header.frame_id="panda_link0";
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

} // end namespace


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "sara_shield_node");
    franka_sara_shield_controller::SaraShieldRosNode node;
    ros::spin();
    return 0;
}