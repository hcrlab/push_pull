#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#ifndef _PR2_PICK_MANIPULATION_DRIVER_H_
#define _PR2_PICK_MANIPULATION_DRIVER_H_

namespace pr2_pick_manipulation {
class RobotDriver {
 private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  //! We will be listening to TF transforms as well
  tf::TransformListener listener_;

 public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh);

  //! Drive forward a specified distance based on odometry information
  bool Drive(geometry_msgs::Twist base_cmd, double distance);
};
};  // namespace pr2_pick_manipulation
#endif
