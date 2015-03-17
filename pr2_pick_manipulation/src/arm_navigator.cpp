#include "geometry_msgs/PoseStamped.h"
#include "pr2_pick_manipulation/arm_navigator.h"
#include "ros/ros.h"
#include "trajopt_test/TrajoptNavigate.h"
#include "moveit/move_group_interface/move_group.h"

using pr2_pick_manipulation::ArmId;

namespace pr2_pick_manipulation {
//ArmNavigator::ArmNavigator(const ros::NodeHandle& node_handle, const ArmId arm_id):
//    node_handle_(node_handle),
//    arm_id_(arm_id) {
//  navigate_client_ = node_handle_.serviceClient<trajopt_test::TrajoptNavigate>("trajopt_navigate");
//};
//
//bool ArmNavigator::MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
//                                  const bool refresh_point_cloud) {
//  trajopt_test::TrajoptNavigateRequest request;
//  request.goal = pose;
//  request.new_cloud = refresh_point_cloud;
//  trajopt_test::TrajoptNavigateResponse response;
//  navigate_client_.call(request, response);
//  return response.success;
//}

ArmNavigator::ArmNavigator(moveit::planning_interface::MoveGroup& group):
    group_(group) {
};

bool ArmNavigator::MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
                                  const bool refresh_point_cloud) {
  group_.setPoseTarget(pose);
  return group_.move();
}

};  // namespace pr2_pick_manipulation
