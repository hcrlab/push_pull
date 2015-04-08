// Run this node to start the move head service.

#include <ros/ros.h>
#include "pr2_pick_manipulation/move_head_service.h"

using pr2_pick_manipulation::MoveHeadService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_head_service_node");
  MoveHeadService service;
  ros::spin();
  return 0;
}
