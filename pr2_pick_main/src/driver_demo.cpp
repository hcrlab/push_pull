#include <ros/ros.h>

#include "pr2_pick_manipulation/driver.h"

using pr2_pick_manipulation::RobotDriver;

/**
 * A simple demonstration of the base driver client declared in
 * pr2_pick_manipulation/driver.h
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "driver_demo");

  RobotDriver driver;

  // Specify the speed in the x and y direction in meters/second as well as
  // the angular velocity in radians/second.
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

  bool status;
  double speed = 0.125;
  double distance = 0.25;

  // Drive right.
  base_cmd.linear.y = -speed;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    base_cmd.linear.y, distance);
  status = driver.Drive(base_cmd, distance);
  printf("status %d\n", status);

  // Drive left, back to the starting position.
  base_cmd.linear.y = speed;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    base_cmd.linear.y, distance);
  status = driver.Drive(base_cmd, distance);
  printf("status %d\n", status);

  return 0;
}
