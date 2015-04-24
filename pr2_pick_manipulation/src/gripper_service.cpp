#include <ros/ros.h>
#include "pr2_pick_manipulation/gripper_service.h"
#include "pr2_pick_manipulation/gripper.h"

namespace pr2_pick_manipulation {
GripperService::GripperService()
    : nh_(),
      set_grippers_server_(nh_.advertiseService("set_grippers_service",
                                   &GripperService::SetGrippersCallback,
                                   this)),
      get_grippers_server_(nh_.advertiseService("get_grippers_service",
                                   &GripperService::GetGrippersCallback,
                                   this)),
      get_gripper_positions_server_(nh_.advertiseService("get_gripper_positions_service",
                                   &GripperService::GetGripperPositionsCallback,
                                   this)),
      left_(Gripper::LEFT_GRIPPER),
      right_(Gripper::RIGHT_GRIPPER) {
}

bool GripperService::SetGrippersCallback(SetGrippers::Request& request,
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

bool GripperService::GetGrippersCallback(GetGrippers::Request& request,
                                         GetGrippers::Response& response) {
  response.left_open = left_.IsOpen();
  response.right_open = right_.IsOpen();
  return true;
}

bool GripperService::GetGripperPositionsCallback(GetGripperPositions::Request& request,
                                                 GetGripperPositions::Response& response) {
  response.left_position = left_.GetPosition();
  response.right_position = right_.GetPosition();
  return true;
}


};  // namespace pr2_pick_manipulation
