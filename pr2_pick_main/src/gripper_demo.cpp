#include <ros/ros.h>
#include <unistd.h>
#include <ctime>

#include "pr2_pick_manipulation/gripper.h"

using pr2_pick_manipulation::Gripper;

/**
 * A simple demonstration of the torso client declared in
 * pr2_pick_manipulation/torso.h. Tests valid and invalid positions.
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "gripper_demo");
  ros::NodeHandle node_handle;

  Gripper gripper("l_gripper_controller/gripper_action");
  double positions[] = {0.03, 0.01, 0.07, 0.1};
  bool status;

  for(int i = 0; i < sizeof(positions) / sizeof(double); i++) {
    printf("Setting gripper position to %0.3f\n", positions[i]);
    status = gripper.setPosition(positions[i]);
    printf("Status: %s\n", status ? "succeeded" : "failed");
  }

  printf("Opening gripper (kOpen = %0.3f)\n", Gripper::kOpen);
  status = gripper.open();
  printf("Status: %s\n", status ? "succeeded" : "failed");

  printf("Closing gripper (kClosed = %0.3f)\n", Gripper::kClosed);
  status = gripper.close();
  printf("Status: %s\n", status ? "succeeded" : "failed");

  return 0;
}
