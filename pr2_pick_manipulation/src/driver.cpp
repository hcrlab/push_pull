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
  
  ros::Rate rate(10.0);
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
  std_msgs::Float64 linearVelocity, std_msgs::Float64 angularVelocity) {
  // Transform pose to be relative to "base_footprint"
  geometry_msgs::PoseStamped newPose;
  listener_.transformPose("base_footprint", pose, newPose);

  // tf::Transform t;
  // t.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y,
  //   pose.pose.position.y));
  // t.setRotation(tf::Quaternion(pose.pose.orientation.w, pose.pose.orientation.x,
  //   pose.pose.orientation.y, pose.pose.orientation.z));
  
  // tf::Vector3 location;
  // tf::Quaternion rotation;
  // (location, rotation) = t.lookupTransform("base_footprint",
  //   pose.header.frame_id, ros::Time::now());

  tf::Matrix3x3 m(newPose.pose.orientation);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Move to point using DriveLinear
  double distance = sqrt((newPose.pose.position.x ^ 2) +
    (newPose.pose.position.y ^ 2));
  double dx = linearVelocity * (newPose.pose.position.x / distance);
  double dy = linearVelocity * (newPose.pose.position.y / distance);
  RobotDriver::DriveLinear(dx, dy, distance);
  // Rotate to final pose using DriveAngular
  RobotDriver::DriveAngular((double)angularVelocity, yaw);
}
};  // namespace pr2_pick_manipulation
