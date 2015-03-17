#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"

#ifndef _PR2_PICK_MANIPULATION_ARM_NAVIGATOR_H_
#define _PR2_PICK_MANIPULATION_ARM_NAVIGATOR_H_

namespace pr2_pick_manipulation {
enum ArmId { kLeft, kRight };

//class ArmNavigator {
// private:
//  ros::NodeHandle node_handle_;   
//  ArmId arm_id_;
//  ros::ServiceClient navigate_client_;
//
// public:
//  ArmNavigator(const ros::NodeHandle& node_handle_, const ArmId arm_id);
//  bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
//                      const bool refresh_point_cloud);
//};

class ArmNavigator {
 private:
  moveit::planning_interface::MoveGroup group_;

 public:
  ArmNavigator(moveit::planning_interface::MoveGroup& group);
  bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
                      const bool refresh_point_cloud);
};


};  // namespace pr2_pick_manipulation

#endif
