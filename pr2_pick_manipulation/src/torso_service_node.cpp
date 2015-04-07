// Run this node to start the torso service.

#include <ros/ros.h>
#include "pr2_pick_manipulation/torso_service.h"

using pr2_pick_manipulation::TorsoService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "torso_service_node");
  TorsoService service;
  ros::spin();
  return 0;
}
