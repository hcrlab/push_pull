// Run this node to start the tuck arms service.

#include <ros/ros.h>
#include "pr2_pick_manipulation/tuck_arms_service.h"

using pr2_pick_manipulation::TuckArmsService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "tuck_arms_service_node");
  TuckArmsService service;
  ros::spin();
  return 0;
}
