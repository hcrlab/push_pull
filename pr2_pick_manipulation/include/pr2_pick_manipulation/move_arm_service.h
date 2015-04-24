// A service for moving the robot's arms.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import MoveArm
// move_arm = rospy.ServiceProxy('move_arm_service', MoveArm)
// move_arm.wait_for_service()
// # Move the right arm to the given goal pose.
// move_arm(goal=PoseStamped(...), planning_time=10, group='right_arm',
//   position_tolerance=0, orientation_tolerance=0)

#include <ros/ros.h>
#include "moveit/move_group_interface/move_group.h"
#include "pr2_pick_manipulation/MoveArm.h"

#ifndef _PR2_PICK_MANIPULATION_MOVE_ARM_SERVICE_H_
#define _PR2_PICK_MANIPULATION_MOVE_ARM_SERVICE_H_

namespace pr2_pick_manipulation {
class MoveArmService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  move_group_interface::MoveGroup left_group_;
  move_group_interface::MoveGroup right_group_;
  bool Callback(MoveArm::Request& request, MoveArm::Response& response);

 public:
  MoveArmService();
};
};  // namespace pr2_pick_manipulation

#endif // _PR2_PICK_MANIPULATION_MOVE_ARM_SERVICE_H_
