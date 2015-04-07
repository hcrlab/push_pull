#include <ros/ros.h>
#include "pr2_pick_manipulation/gripper_service.h"

using pr2_pick_manipulation::GripperService;

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper_service_node");
  GripperService service;
  ros::spin();
  return 0;
}
