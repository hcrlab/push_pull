// A library for driving the robot using the low-level base controller and
// odometry.
//
// Sample usage:
//  RobotDriver driver;
//  // Drive left for 0.25 meters at 0.125 m/s
//  driver.DriveLinear(0, 0.125, 0.25);
//  // Rotate counter-clockwise 90 degrees.
//  driver.DriveAngular(-0.5, M_PI/2);
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#ifndef _PR2_PICK_MANIPULATION_DRIVER_H_
#define _PR2_PICK_MANIPULATION_DRIVER_H_

namespace pr2_pick_manipulation {
class RobotDriver {
 private:
  ros::NodeHandle node_handle_;
  ros::Publisher cmd_vel_pub_;
  tf::TransformListener listener_;

 public:
  RobotDriver();

  // Drives the robot in a straight line.
  //
  // dx and dy are linear velocities (+dx forward, +dy left) in meters/second
  //
  // Stops driving once the robot has been displaced by the given distance from
  // its starting position.
  bool DriveLinear(double dx, double dy, double distance);

  // Rotates the robot.
  //
  // dt is an angular velocity (+dt counter-clockwise) in radians/second
  //
  // Stops rotating once the robot has rotated the specified number of radians.
  bool DriveAngular(double dt, double radians);
};
};  // namespace pr2_pick_manipulation
#endif
