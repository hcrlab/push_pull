#include <ros/ros.h>
#include "pr2_pick_manipulation/gripper_service.h"
#include "pr2_pick_manipulation/gripper.h"

namespace pr2_pick_manipulation {
GripperService::GripperService()
    : nh_(),
      server_(nh_.advertiseService("gripper_service",
                                   &GripperService::Callback,
                                   this)),
      left_(Gripper::LEFT_GRIPPER),
      right_(Gripper::RIGHT_GRIPPER) {
}

bool GripperService::Callback(SetGrippers::Request& request,
                              SetGrippers::Response& response) {
  if (request.open_left) {
    left_.Open(-1);
  } else {
    left_.Close(-1); 
  }
  if (request.open_right) {
    right_.Open(-1);
  } else {
    right_.Close(-1); 
  }
  response.success = true;
  return true;
}

};  // namespace pr2_pick_manipulation
