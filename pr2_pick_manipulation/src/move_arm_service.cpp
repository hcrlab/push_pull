#include <ros/ros.h>
#include "moveit/move_group_interface/move_group.h"
#include "pr2_pick_manipulation/move_arm_service.h"

using move_group_interface::MoveGroup;

namespace pr2_pick_manipulation {
MoveArmService::MoveArmService()
  : nh_(),
    server_(nh_.advertiseService("move_arm_service",
                                 &MoveArmService::Callback,
                                 this)),
    left_group_("left_arm"),
    right_group_("right_arm") {
}

bool MoveArmService::Callback(MoveArm::Request& request,
                              MoveArm::Response& response) {
  MoveGroup& group = right_group_;
  if (request.group == "left_arm") {
    group = left_group_;
  } else if (request.group == "right_arm") {
  } else {
    return false;
  }

  group.setPoseTarget(request.goal);
  double planning_time = request.planning_time;
  if (planning_time == 0) {
    planning_time = 10;
  }
  group.setPlanningTime(planning_time);
  if (request.position_tolerance > 0) {
    group.setGoalPositionTolerance(request.position_tolerance);
  }
  if (request.orientation_tolerance > 0) {
    group.setGoalOrientationTolerance(request.orientation_tolerance);
  }

  bool success = group.move();
  response.success = success;
  return success;
}
};  // namespace pr2_pick_manipulation
