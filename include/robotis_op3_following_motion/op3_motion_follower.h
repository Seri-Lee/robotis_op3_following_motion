#ifndef OP3_MOTION_FOLLOWER_H
#define OP3_MOTION_FOLLOWER_H

#define EIGEN_NO_DEBUG
#define EIGEN_NO_STATIC_ASSERT

#include <cmath>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include "openpose_ros_msgs/Persons.h"
#include "robotis_math/robotis_linear_algebra.h"
#include "robotis_controller_msgs/GetJointModule.h"

namespace robotis_op
{

class MotionFollower
{
 public:

  MotionFollower();
  ~MotionFollower();

  // const

  // enum

  // method

  // variable

 protected:

  // const
  const std::string MODULE_NAME;
  const bool DEBUG_PRINT;

  // enum
  enum BodyParts
  {
    Nose = 0,
    Neck = 1,
    RShoulder = 2,
    RElbow = 3,
    RWrist = 4,
    LShoulder = 5,
    LElbow = 6,
    LWrist = 7,
    RHip = 8,
    RKnee = 9,
    RAnkle = 10,
    LHip = 11,
    LKnee = 12,
    LAnkle = 13,
    REye = 14,
    LEye = 15,
    REar = 16,
    LEar = 17
  };

  // method
  void humanPoseCallback(const openpose_ros_msgs::Persons::ConstPtr& msg);
  void buttonCallback(const std_msgs::String::ConstPtr& msg);

  void checkTorque();
  void goInitPose();
  void handleModule();
  void setModule(const std::string &module_name);
  void handlePlaying();
  void parseInit();
  void setBaseInitPose();
  bool getShoulderLength(const openpose_ros_msgs::PersonDetection &person, double &length);
  void calcJointStates(const openpose_ros_msgs::PersonDetection &person_to_follow);
  void publishJointStates();

  bool calcJointAngle(Eigen::Vector2d upper, Eigen::Vector2d lower, double &target_angle);

  void checkMaxAngle(const double maximun, double &target_angle);

  //ros node handle
  ros::NodeHandle nh_;

  //image publisher/subscriber
  ros::Subscriber human_pose_sub_;
  ros::Subscriber button_sub_;
  ros::Publisher op3_joints_pub_;
  ros::Publisher dxl_torque_pub_;
  ros::Publisher set_module_pub_;
  ros::Publisher init_pose_pub_;

  ros::ServiceClient get_module_client_;

  // variable
  openpose_ros_msgs::PersonDetection person_to_follow_;
  std::map <int, Eigen::Vector2d> body_position_;
  std::map<std::string, double> joint_angles_;

  Eigen::Vector3d l_shoulder_3d_, r_shoulder_3d_;

  bool is_ready_;
  bool is_playable_;
};
}

#endif // OP3_MOTION_FOLLOWER_H
