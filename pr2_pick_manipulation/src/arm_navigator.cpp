#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
#include "pr2_pick_manipulation/arm_navigator.h"
#include "ros/ros.h"
#include "trajopt_test/TrajoptNavigate.h"

using geometry_msgs::PoseStamped;
using moveit::planning_interface::MoveGroup;
using pr2_pick_manipulation::ArmId;
using ros::ServiceClient;

namespace pr2_pick_manipulation {
MoveItArmNavigator::MoveItArmNavigator(MoveGroup& group): group_(group) {
}

bool MoveItArmNavigator::MoveToPoseGoal(const PoseStamped& pose,
                                        const bool refresh_point_cloud) {
  group_.setPoseTarget(pose);
  return group_.move();
}

TrajOptArmNavigator::TrajOptArmNavigator(const ArmId arm_id,
                                         const ServiceClient& client)
    : arm_id_(arm_id),
      client_(client) {
}

bool TrajOptArmNavigator::MoveToPoseGoal(const PoseStamped& pose,
                                         const bool refresh_point_cloud) {
  trajopt_test::TrajoptNavigateRequest request;
  request.goal = pose;
  request.new_cloud = refresh_point_cloud;
  trajopt_test::TrajoptNavigateResponse response;
  client_.call(request, response);
  return response.success;
}
};  // namespace pr2_pick_manipulation
