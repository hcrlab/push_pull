// Run this node to start the move arm service.

#include <ros/ros.h>
#include "pr2_pick_manipulation/move_arm_service.h"

using pr2_pick_manipulation::MoveArmService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_arm_service_node");
  MoveArmService service;
  ros::spin();
  return 0;
}
