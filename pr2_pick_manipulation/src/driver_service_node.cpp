// Run this node to start the driver services.

#include <ros/ros.h>
#include "pr2_pick_manipulation/driver_service.h"

using pr2_pick_manipulation::DriverService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_service_node");
  DriverService service;
  ros::spin();
  return 0;
}
