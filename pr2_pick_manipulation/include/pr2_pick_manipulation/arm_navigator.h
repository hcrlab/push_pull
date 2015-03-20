#include "geometry_msgs/PoseStamped.h"
#include "moveit/move_group_interface/move_group.h"
#include "ros/ros.h"

#ifndef _PR2_PICK_MANIPULATION_ARM_NAVIGATOR_H_
#define _PR2_PICK_MANIPULATION_ARM_NAVIGATOR_H_

namespace pr2_pick_manipulation {
enum ArmId { kLeft, kRight };

// Interface for sending arm navigation goals.
//
// Sample usage:
//   MoveGroup group("right_arm");
//   ArmNavigatorInterface* right_arm = new MoveItArmNavigator(group);
//   right_arm.MoveToPoseGoal(pose, false);
class ArmNavigatorInterface {
 public:
  ArmNavigatorInterface() {};
  virtual ~ArmNavigatorInterface() {}

  // Sends a goal for the end effector to move to the given pose.
  // refresh_point_cloud tells trajopt to capture a new point cloud before
  // planning. Otherwise, it uses a previously captured point cloud. This arg
  // has not effect on MoveIt.
  virtual bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
                              const bool refresh_point_cloud) = 0;
};

// Implements arm navigation using MoveIt.
class MoveItArmNavigator : public ArmNavigatorInterface {
 private:
  moveit::planning_interface::MoveGroup group_;

 public:
  MoveItArmNavigator(moveit::planning_interface::MoveGroup& group);
  ~MoveItArmNavigator() {};
  bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
                      const bool refresh_point_cloud);
};

// Implements arm navigation using trajopt.
class TrajOptArmNavigator : public ArmNavigatorInterface {
 private:
  ArmId arm_id_;
  ros::ServiceClient client_;
 
 public:
  TrajOptArmNavigator(const ArmId arm_id, const ros::ServiceClient& client);
  ~TrajOptArmNavigator() {};
  bool MoveToPoseGoal(const geometry_msgs::PoseStamped& pose,
                      const bool refresh_point_cloud);
};
};  // namespace pr2_pick_manipulation

#endif
