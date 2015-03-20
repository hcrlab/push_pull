// A library for driving the robot using the low-level base controller and
// odometry.
//
// Sample usage:
//  ros::NodeHandle node_handle;
//  pr2_pick_manipulation::RobotDriver driver(node_handle);  
//
//  // Specify the speed in the x and y direction in meters/second as well as
//  // the angular velocity in radians/second.
//  geometry_msgs::Twist base_cmd;
//  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
//  base_cmd.linear.y = 0.125;
//
//  // Drive until the robot has been displaced 0.25 meters from its starting
//  // position.
//  driver.Drive(base_cmd, 0.25);
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#ifndef _PR2_PICK_MANIPULATION_DRIVER_H_
#define _PR2_PICK_MANIPULATION_DRIVER_H_

namespace pr2_pick_manipulation {
class RobotDriver {
 private:
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;

 public:
  RobotDriver(const ros::Publisher& cmd_vel_pub);

  // Drive with the velocities given in base_cmd.linear.x (+x forward),
  // base_cmd.linear.y (+y left), and base_cmd.angular.z (+z clockwise), with
  // units of meters/second and radians/second.
  //
  // Stops driving once the robot has been displaced by the given distance from
  // its starting position.
  //
  // Note that this method is mostly suited for driving linearly. If you give it
  // just an angular velocity, then it will spin around forever.
  bool Drive(geometry_msgs::Twist base_cmd, double distance);
};
};  // namespace pr2_pick_manipulation
#endif
