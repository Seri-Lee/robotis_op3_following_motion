#include "robotis_op3_following_motion/op3_motion_follower.h"

namespace robotis_op
{
MotionFollower::MotionFollower()
  : nh_(ros::this_node::getName()),
    MODULE_NAME("direct_control_module"),
    l_shoulder_3d_(0, 0, 0),
    is_ready_(false),
    is_playable_(false),
    DEBUG_PRINT(false)
{
  human_pose_sub_ = nh_.subscribe("/openpose/pose", 1, &MotionFollower::humanPoseCallback, this);
  button_sub_ = nh_.subscribe("/robotis/open_cr/button", 1, &MotionFollower::buttonCallback, this);
  op3_joints_pub_ = nh_.advertise<sensor_msgs::JointState>("/robotis/direct_control/set_joint_states", 0);
  dxl_torque_pub_ = nh_.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  set_module_pub_ = nh_.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 0);
  init_pose_pub_ = nh_.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);

  get_module_client_ = nh_.serviceClient<robotis_controller_msgs::GetJointModule>(
      "/robotis/get_present_joint_ctrl_modules");

  joint_angles_.clear();
}

MotionFollower::~MotionFollower()
{

}

void MotionFollower::humanPoseCallback(const openpose_ros_msgs::Persons::ConstPtr& msg)
{
  if(is_ready_ == false || is_playable_ == false)
    return;

  // check human pose data
  if(msg->persons.size() == 0)
    return;

  // calc joint angle of ROBOTIS-OP3 to follow the motion of person who is the closest
  int index_of_closest_person = -1;
  double shoulder_size = 0.0;
  for(int ix = 0; ix < msg->persons.size(); ix++)
  {
    double s_length = 0.0;
    // get length : shoulder + neck
    bool result = getShoulderLength(msg->persons[ix], s_length);
    // check the largest
    if(result == true && s_length > shoulder_size)
    {
      shoulder_size = s_length;
      index_of_closest_person = ix;
    }
  }

  if(index_of_closest_person == -1)
    return;

  ROS_INFO_STREAM("Pose message[" << msg->persons.size() << "] : " << index_of_closest_person);
  calcJointStates(msg->persons[index_of_closest_person]);

  // publish joint angle
  publishJointStates();
}

void MotionFollower::buttonCallback(const std_msgs::String::ConstPtr& msg)
{
  // if mode button is pressed, OP3 will torque on and go init pose
  if(msg->data == "mode")
  {
    // check torque
    checkTorque();

    // handle Module
    handleModule();

    // go init pose and start following demo
    goInitPose();

    is_playable_ = true;
  }
  else if(msg->data == "start")
  {
    // start and pause
    handlePlaying();

  }
  else if(msg->data == "user")
  {
    // go init of base_module
    setBaseInitPose();

    is_ready_ = false;
    is_playable_ = false;
  }
}

void MotionFollower::checkTorque()
{
  std_msgs::String check_msg;
  check_msg.data = "check";

  // send command to op3_manager in order to check the torque condition of OP3
  dxl_torque_pub_.publish(check_msg);

  // wait for checking torque
  usleep(40 * 1000);
}

void MotionFollower::goInitPose()
{
  // go init pose from yaml file
  parseInit();

  publishJointStates();

  // wait for goint to init pose.
  usleep(3 * 1000 * 1000);

  // set ready flag
  is_ready_ = true;
}

void MotionFollower::handleModule()
{
  // set module to DIRECT_CONTROL
  setModule(MODULE_NAME);

  // wait to setting completed
  usleep(20 * 1000);
}

void MotionFollower::setModule(const std::string &module_name)
{
  // set module to direct_control_module for this demonsration
  std_msgs::String module_msg;
  module_msg.data = module_name;

  set_module_pub_.publish(module_msg);
}

void MotionFollower::handlePlaying()
{
  if(is_ready_ == false)
    return;

  is_playable_ = !is_playable_;
}

void MotionFollower::parseInit()
{
  joint_angles_.clear();
  std::string init_pose_path = ros::package::getPath("robotis_op3_following_motion") + "/data/ini_pose.yaml";

  if(init_pose_path == "")
    return;

  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(init_pose_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator yaml_it = tar_pose_node.begin(); yaml_it != tar_pose_node.end(); ++yaml_it)
  {
    std::string joint_name;
    double value;

    joint_name = yaml_it->first.as<std::string>();
    value = yaml_it->second.as<double>();

    joint_angles_[joint_name] = value * M_PI / 180.0;
  }
}

void MotionFollower::setBaseInitPose()
{
  std_msgs::String init_msg;
  init_msg.data = "ini_pose";

  init_pose_pub_.publish(init_msg);
}

bool MotionFollower::getShoulderLength(const openpose_ros_msgs::PersonDetection &person, double& length)
{
  Eigen::Vector2d neck(0.0, 0.0), r_shoulder(0.0, 0.0), l_shoulder(0.0, 0.0), nose(0.0, 0.0);
  double r_sho_len = 0.0, l_sho_len = 0.0, nose_len = 0.0;

  for(int ix = 0; ix < person.body_part.size(); ix++)
  {
    openpose_ros_msgs::BodyPartDetection body_part = person.body_part[ix];
    if(body_part.part_id == Neck)
    {
      if(body_part.confidence == 0.0)
        return false;
      neck = Eigen::Vector2d(body_part.x, body_part.y);
    }
    if(body_part.part_id == RShoulder)
    {
      r_shoulder = Eigen::Vector2d(body_part.x, body_part.y);
    }
    if(body_part.part_id == LShoulder)
    {
      l_shoulder = Eigen::Vector2d(body_part.x, body_part.y);
    }
    if(body_part.part_id == Nose)
    {
      nose = Eigen::Vector2d(body_part.x, body_part.y);
    }
  }

  if(r_shoulder != Eigen::Vector2d(0.0, 0.0))
  {
    Eigen::Vector2d r_len_vec = r_shoulder - neck;
    r_sho_len = r_len_vec.norm();
  }
  if(l_shoulder != Eigen::Vector2d(0.0, 0.0))
  {
    Eigen::Vector2d l_len_vec = l_shoulder - neck;
    l_sho_len = l_len_vec.norm();
  }
  if(nose != Eigen::Vector2d(0.0, 0.0))
  {
    Eigen::Vector2d nose_len_vec = nose - neck;
    nose_len = nose_len_vec.norm();
  }

  length = r_sho_len + l_sho_len + nose_len;
  return true;
}

void MotionFollower::calcJointStates(const openpose_ros_msgs::PersonDetection &person_to_follow)
{
  joint_angles_.clear();

  ROS_INFO("Get person data");
  // store body position
  for(int ix = 0; ix < person_to_follow.body_part.size(); ix++)
  {
    openpose_ros_msgs::BodyPartDetection body_part = person_to_follow.body_part[ix];
    body_position_[body_part.part_id] = Eigen::Vector2d(body_part.x, body_part.y);
  }

  bool calc_result = false;

  // calc left shoulder
  double neck_to_shoulder_ratio = 11.0 / 7.0;

  if(body_position_[Neck] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LElbow] != Eigen::Vector2d(0.0, 0.0))
  {
    double l_shoulder_roll = 0.0, l_shoulder_pitch = 0.0;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck : (" << body_position_[Neck].coeff(0) << ", " << body_position_[Neck].coeff(1)
                         << "), LShoulder : (" << body_position_[LShoulder].coeff(0) << ", " << body_position_[LShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[LElbow].coeff(0) << ", " << body_position_[LElbow].coeff(1) << ")");

    Eigen::Vector2d neck_vector = body_position_[Neck] - body_position_[LShoulder];
    Eigen::Vector2d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector2d arm_vector = body_position_[Neck] - body_position_[LElbow];

    double neck_len = neck_vector.norm();
    double shoulder_len_target = neck_len * neck_to_shoulder_ratio;
    double shoulder_len = shoulder_vector.norm();
    if(shoulder_len > shoulder_len_target) shoulder_len_target = shoulder_len;

    ROS_INFO_COND(DEBUG_PRINT, "==============================================");
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck length : " << neck_len << ", Shoulder length : " << shoulder_len);

    double l_shoulder_angle = acos(shoulder_len / shoulder_len_target);
    Eigen::Vector3d should_3d_vec = Eigen::Vector3d(sin(l_shoulder_angle) * shoulder_len_target, shoulder_vector[0], shoulder_vector[1]);


    double alpha = 0.75;
    if(l_shoulder_3d_ == Eigen::Vector3d(0, 0, 0)) alpha = 0;
    l_shoulder_3d_ = l_shoulder_3d_ * alpha + should_3d_vec * (1 - alpha);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder 3D Vector : [alpha - " << alpha << "]\n" << l_shoulder_3d_);

    Eigen::Vector3d toward(0, 1, 0);
    should_3d_vec.normalize();
    Eigen::Quaterniond shoulder_orientation(Eigen::Quaterniond::FromTwoVectors(toward, should_3d_vec));
    Eigen::Vector3d shoulder_rpy = robotis_framework::convertRotationToRPY(shoulder_orientation.toRotationMatrix());

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder Roll : " << (shoulder_rpy[0] * 180.0 / M_PI) << ", Pitch : " << (shoulder_rpy[1] * 180.0 / M_PI) << ", Yaw : "  << (shoulder_rpy[2] * 180.0 / M_PI));

    // adjustment shoulder


    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, l_shoulder_roll);

    Eigen::Vector3d shoulder_vector_3d, arm_vector_3d;
    shoulder_vector_3d << shoulder_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d shoulder_direction = shoulder_vector_3d.cross(arm_vector_3d);

    if(shoulder_direction.coeff(2) > 0)
      l_shoulder_roll *= (-1);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Left_Shoulder: [Roll] " << (l_shoulder_roll * 180 / M_PI) << ", [Pitch] " << (l_shoulder_pitch * 180 / M_PI));

    checkMaxAngle(80 * M_PI / 180.0, l_shoulder_roll);
    joint_angles_["l_sho_pitch"] = l_shoulder_pitch;
    joint_angles_["l_sho_roll"] = l_shoulder_roll;
  }
  else
  {
    l_shoulder_3d_ = Eigen::Vector3d(0, 0, 0);
  }

  // calc right shoulder
  if(body_position_[Neck] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RElbow] != Eigen::Vector2d(0.0, 0.0))
  {
    double r_shoulder_roll = 0.0, r_shoulder_pitch = 0.0;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck : (" << body_position_[Neck].coeff(0) << ", " << body_position_[Neck].coeff(1)
                         << "), RShoulder : (" << body_position_[RShoulder].coeff(0) << ", " << body_position_[RShoulder].coeff(1)
                         << "), RElbow : (" << body_position_[RElbow].coeff(0) << ", " << body_position_[RElbow].coeff(1) << ")");

    Eigen::Vector2d neck_vector = body_position_[Neck] - body_position_[RShoulder];
    Eigen::Vector2d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector2d arm_vector = body_position_[Neck] - body_position_[RElbow];

    double neck_len = neck_vector.norm();
    double shoulder_len_target = neck_len * neck_to_shoulder_ratio;
    double shoulder_len = shoulder_vector.norm();
    if(shoulder_len > shoulder_len_target) shoulder_len_target = shoulder_len;

    ROS_INFO_COND(DEBUG_PRINT, "==============================================");
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Neck length : " << neck_len << ", Shoulder length : " << shoulder_len);

    double r_shoulder_angle = acos(shoulder_len / shoulder_len_target);
    Eigen::Vector3d should_3d_vec = Eigen::Vector3d(sin(r_shoulder_angle) * shoulder_len_target, shoulder_vector[0], shoulder_vector[1]);


    double alpha = 0.75;
    if(r_shoulder_3d_ == Eigen::Vector3d(0, 0, 0)) alpha = 0;
    r_shoulder_3d_ = r_shoulder_3d_ * alpha + should_3d_vec * (1 - alpha);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder 3D Vector : [alpha - " << alpha << "]\n" << r_shoulder_3d_);

    Eigen::Vector3d toward(0, 1, 0);
    should_3d_vec.normalize();
    Eigen::Quaterniond shoulder_orientation(Eigen::Quaterniond::FromTwoVectors(toward, should_3d_vec));
    Eigen::Vector3d shoulder_rpy = robotis_framework::convertRotationToRPY(shoulder_orientation.toRotationMatrix());

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Shoulder Roll : " << (shoulder_rpy[0] * 180.0 / M_PI) << ", Pitch : " << (shoulder_rpy[1] * 180.0 / M_PI) << ", Yaw : "  << (shoulder_rpy[2] * 180.0 / M_PI));

    // adjustment shoulder


    // get a shoulder pitch angle
    calc_result = calcJointAngle(neck_vector, shoulder_vector, r_shoulder_roll);

    Eigen::Vector3d shoulder_vector_3d, arm_vector_3d;
    shoulder_vector_3d << shoulder_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d shoulder_direction = shoulder_vector_3d.cross(arm_vector_3d);

    if(shoulder_direction.coeff(2) > 0)
      r_shoulder_roll *= (-1);

    //if(calc_result == true)
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Right_Shoulder: [Roll] " << (r_shoulder_roll * 180 / M_PI) << ", [Pitch] " << (r_shoulder_pitch * 180 / M_PI));

    checkMaxAngle(80 * M_PI / 180.0, r_shoulder_roll);
    joint_angles_["r_sho_pitch"] = r_shoulder_pitch;
    joint_angles_["r_sho_roll"] = r_shoulder_roll;
  }
  else
  {
    r_shoulder_3d_ = Eigen::Vector3d(0, 0, 0);
  }

  // calc left arm

  if(body_position_[LShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LElbow] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[LWrist] != Eigen::Vector2d(0.0, 0.0))
  {
    double l_elbow = 0.0;
    bool is_upside_left = false;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "LShoulder : (" << body_position_[LShoulder].coeff(0) << ", " << body_position_[LShoulder].coeff(1)
                         << "), LElbow : (" << body_position_[LElbow].coeff(0) << ", " << body_position_[LElbow].coeff(1)
                         << "), LWrist : (" << body_position_[LWrist].coeff(0) << ", " << body_position_[LWrist].coeff(1) << ")");

    Eigen::Vector2d shoulder_vector = body_position_[LShoulder] - body_position_[LElbow];
    Eigen::Vector2d elbow_vector = body_position_[LElbow] - body_position_[LWrist];
    Eigen::Vector2d arm_vector = body_position_[LShoulder] - body_position_[LWrist];
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "L-Elbow : (" << elbow_vector.coeff(0) << ", " << elbow_vector.coeff(1) << ")");

    calc_result = calcJointAngle(shoulder_vector,elbow_vector, l_elbow);

    Eigen::Vector3d elbow_vector_3d, arm_vector_3d;
    elbow_vector_3d << elbow_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d elbow_direction = elbow_vector_3d.cross(arm_vector_3d);

    if(elbow_direction.coeff(2) < 0)
      l_elbow *= (-1);

    if(is_upside_left == true)
    {
      std::map<std::string, double>::iterator joints_it;

      joints_it = joint_angles_.find("l_sho_pitch");
      if (joints_it != joint_angles_.end())
        joint_angles_["l_sho_pitch"] = M_PI - joint_angles_["l_sho_pitch"];
      else
        joint_angles_["l_sho_pitch"] = M_PI;

      joints_it = joint_angles_.find("l_sho_roll");
      if (joints_it != joint_angles_.end())
        joint_angles_["l_sho_pitch"] = - joint_angles_["l_sho_pitch"];
    }

    checkMaxAngle(120 * M_PI / 180.0, l_elbow);
    joint_angles_["l_el"] = l_elbow;
  }

  // calc right arm

  if(body_position_[RShoulder] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RElbow] != Eigen::Vector2d(0.0, 0.0) &&
     body_position_[RWrist] != Eigen::Vector2d(0.0, 0.0))
  {
    double r_elbow = 0.0;
    calc_result = false;

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "RShoulder : (" << body_position_[RShoulder].coeff(0) << ", " << body_position_[RShoulder].coeff(1)
                         << "), RElbow : (" << body_position_[RElbow].coeff(0) << ", " << body_position_[RElbow].coeff(1)
                         << "), RWrist : (" << body_position_[RWrist].coeff(0) << ", " << body_position_[RWrist].coeff(1) << ")");

    Eigen::Vector2d shoulder_vector = body_position_[RShoulder] - body_position_[RElbow];
    Eigen::Vector2d elbow_vector = body_position_[RElbow] - body_position_[RWrist];
    Eigen::Vector2d arm_vector = body_position_[RShoulder] - body_position_[RWrist];
    ROS_INFO_STREAM_COND(DEBUG_PRINT, "R-Elbow : (" << elbow_vector.coeff(0) << ", " << elbow_vector.coeff(1) << ")");

    calc_result = calcJointAngle(shoulder_vector, elbow_vector, r_elbow);

    Eigen::Vector3d elbow_vector_3d, arm_vector_3d;
    elbow_vector_3d << elbow_vector , 0;
    arm_vector_3d << arm_vector, 0;
    Eigen::Vector3d elbow_direction = elbow_vector_3d.cross(arm_vector_3d);

    if(elbow_direction.coeff(2) < 0)
      r_elbow *= (-1);

    checkMaxAngle(120 * M_PI / 180.0, r_elbow);
    joint_angles_["r_el"] = r_elbow;
  }

  // calc head : it will not move

  // calc lower body : it will not move

}

void MotionFollower::publishJointStates()
{
  if(joint_angles_.size() == 0)
    return;

  sensor_msgs::JointState op3_joints;

  for (std::map<std::string, double>::iterator pub_joint_it = joint_angles_.begin(); pub_joint_it != joint_angles_.end(); ++pub_joint_it)
  {
    op3_joints.name.push_back(pub_joint_it->first);
    op3_joints.position.push_back(pub_joint_it->second);
    ROS_INFO_STREAM(pub_joint_it->first << " : " << (pub_joint_it->second * 180 / M_PI));
  }

  // publish jointstates
  if(op3_joints.name.size() != 0)
    op3_joints_pub_.publish(op3_joints);
}

bool MotionFollower::calcJointAngle(Eigen::Vector2d upper, Eigen::Vector2d lower, double& target_angle)
{
  // no data
  if(upper == -lower)
    return false;

  // calc
  double inter_value = upper.dot(lower) / (upper.norm() * lower.norm());
  target_angle = acos(inter_value);

  return true;
}


void MotionFollower::checkMaxAngle(const double maximun, double &target_angle)
{
  if(fabs(target_angle) > maximun)
    target_angle = target_angle > 0 ? maximun : - maximun;
}

}
