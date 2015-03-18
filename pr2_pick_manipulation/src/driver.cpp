#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include "pr2_pick_manipulation/driver.h"

namespace pr2_pick_manipulation {
RobotDriver::RobotDriver(const ros::NodeHandle& nh): nh_(nh) {
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
}

bool RobotDriver::Drive(geometry_msgs::Twist base_cmd, double distance) {
  listener_.waitForTransform("base_footprint", "odom_combined", 
                             ros::Time(0), ros::Duration(1.0));
  
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  listener_.lookupTransform("base_footprint", "odom_combined", 
                            ros::Time(0), start_transform);
  
  ros::Rate rate(10.0);
  bool done = distance == 0;
  while (!done && nh_.ok()) {
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    try {
      listener_.lookupTransform("base_footprint", "odom_combined", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      break;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    double dist_moved = relative_transform.getOrigin().length();

    if(dist_moved > distance) {
      done = true;
    }
  }
  if (done) {
    return true;
  }
  return false;
}
};  // namespace pr2_pick_manipulation
