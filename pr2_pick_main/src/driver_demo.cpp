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

  // Specify the speed in the x and y direction in meters/second.
  // Specify the rotation speed in radians / second.
  
  double dx, dy, dt;
  bool status;
  double rotation_speed = 5 * 2 * M_PI / 60.0; // 5 rotations / minute
  double angle = M_PI / 4.0; // 45 degrees
  double speed = 0.125;
  double distance = 0.25;

  // Drive right.
  dx = 0;
  dy = -speed;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    dy, distance);
  status = driver.DriveLinear(dx, dy, distance);
  printf("status %d\n", status);

  // Drive left, back to the starting position.
  dx = 0;
  dy = speed;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    dy, distance);
  status = driver.DriveLinear(dx, dy, distance);
  printf("status %d\n", status);

  // Drive backward.
  dx = -speed;
  dy = 0;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    dy, distance);
  status = driver.DriveLinear(dx, dy, distance);
  printf("status %d\n", status);

  // Drive forward, back to the starting position.
  dx = speed;
  dy = 0;
  printf("Translating along y-axis at %f meters/second to a distance %f meters\n",
    dy, distance);
  status = driver.DriveLinear(dx, dy, distance);
  printf("status %d\n", status);

  // Rotate counter-clockwise
  dt = rotation_speed;
  printf("Rotating counter-clockwise at %f radians/second to an angle of %f radians\n",
    dt, angle);
  status = driver.DriveAngular(dt, angle);
  printf("status %d\n", status);

  // Rotate counter-clockwise
  dt = -rotation_speed;
  printf("Rotating clockwise at %f radians/second to an angle of %f radians\n",
    dt, angle);
  status = driver.DriveAngular(dt, angle);
  printf("status %d\n", status);
  
  return 0;
}
