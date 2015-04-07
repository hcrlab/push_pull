// A service for opening and closing the grippers.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import Grippers
// rospy.wait_for_service('gripper_service')
// change_grippers = rospy.ServiceProxy('gripper_service', Grippers)
// change_grippers(false, true) # Close left hand and open right hand.

#include "pr2_pick_manipulation/gripper.h"
#include "pr2_pick_manipulation/Grippers.h"
#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_GRIPPER_SERVICE_H_
#define _PR2_PICK_MANIPULATION_GRIPPER_SERVICE_H_

namespace pr2_pick_manipulation {
class GripperService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  Gripper left_;
  Gripper right_;
  bool Callback(Grippers::Request& request, Grippers::Response& response);

 public:
  GripperService();
};
};  // namespace pr2_pick_manipulation

#endif
