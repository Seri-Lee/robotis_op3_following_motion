#include <ros/ros.h>
#include "robotis_op3_following_motion/op3_motion_follower.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op3_following_motion_node");
  ros::NodeHandle nh;

  robotis_op::MotionFollower *op3_motion_follower = new robotis_op::MotionFollower();

  ROS_INFO("Start following motion demo!");

  ros::spin();

}
