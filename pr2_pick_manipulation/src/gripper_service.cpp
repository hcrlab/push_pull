#include <ros/ros.h>
#include "pr2_pick_manipulation/gripper_service.h"
#include "pr2_pick_manipulation/gripper.h"

namespace pr2_pick_manipulation {
GripperService::GripperService()
    : nh_(),
      server_(nh_.advertiseService("gripper_service",
                                   &GripperService::Callback,
                                   this)),
      left_("l_gripper_controller/gripper_action"),
      right_("r_gripper_controller/gripper_action") {
}

bool GripperService::Callback(Grippers::Request& request,
                              Grippers::Response& response) {
  if (request.open_left) {
    left_.open();
  } else {
    left_.close(); 
  }
  if (request.open_right) {
    right_.open();
  } else {
    right_.close(); 
  }
  response.success = true;
  return true;
}

};  // namespace pr2_pick_manipulation
