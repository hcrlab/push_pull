#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>
#include "pr2_pick_manipulation/driver.h"

namespace pr2_pick_manipulation {
RobotDriver::RobotDriver()
    : node_handle_(),
      listener_() {
  cmd_vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>(
    "base_controller/command", 1);
}

bool RobotDriver::DriveLinear(double dx, double dy, double distance) {
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = dx;
  base_cmd.linear.y = dy;
  base_cmd.angular.z = 0;

  listener_.waitForTransform("base_footprint", "odom_combined", 
                             ros::Time(0), ros::Duration(1.0));
  
  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  listener_.lookupTransform("base_footprint", "odom_combined", 
                            ros::Time(0), start_transform);
  
  ros::Rate rate(20.0);
  bool done = distance <= 0;
  while (!done) {
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    try {
      listener_.lookupTransform("base_footprint", "odom_combined", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    double dist_moved = relative_transform.getOrigin().length();

    if(dist_moved > distance) {
      done = true;
    }
  }
  return true;
}

bool RobotDriver::DriveAngular(double dt, double radians) {
  // We still need to test whether "odom_combined"
  // can represent rotations of abs >= 2pi radians.
  if (std::abs(dt) >= 2*M_PI) {
    ROS_ERROR("DriveAngular not tested for abs(dt) > 2pi radians,"
      "aborting");
    return false;
  }

  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = 0;
  base_cmd.linear.y = 0;
  base_cmd.angular.z = dt;

  tf::StampedTransform start_transform;
  tf::StampedTransform current_transform;

  listener_.lookupTransform("base_footprint", "odom_combined", 
                            ros::Time(0), start_transform);
  
  ros::Rate rate(20.0);
  bool done = radians == 0;
  while (!done) {
    cmd_vel_pub_.publish(base_cmd);
    rate.sleep();
    try {
      listener_.lookupTransform("base_footprint", "odom_combined", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    double angle_moved = relative_transform.getRotation().getAngle();

    if(angle_moved > radians) {
      done = true;
    }
  }
  return true;
}
bool RobotDriver::DriveToPose(geometry_msgs::PoseStamped pose,
  double linearVelocity, double angularVelocity) {
  // Transform pose to be relative to "base_footprint"
  geometry_msgs::PoseStamped newPose;
  listener_.transformPose("base_footprint", pose, newPose);

  tf::Matrix3x3 m(tf::Quaternion(newPose.pose.orientation.x, 
                                 newPose.pose.orientation.y, 
                                 newPose.pose.orientation.z,
                                 newPose.pose.orientation.w));
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Move to point using DriveLinear
  double posX = newPose.pose.position.x;
  double posY = newPose.pose.position.y;
  double distance = sqrt(pow(newPose.pose.position.x, 2) +
    pow(newPose.pose.position.y, 2));
  double dx = linearVelocity * (posX / distance);
  double dy = linearVelocity * (posY / distance);
  
  bool success = RobotDriver::DriveLinear(dx, dy, distance);
  
  // Rotate to final pose using DriveAngular
  if (yaw < 0) {
    angularVelocity *= -1;
    yaw *= -1;
  }
  success = success && RobotDriver::DriveAngular((double)angularVelocity, yaw);

  return success;
}
};  // namespace pr2_pick_manipulation
