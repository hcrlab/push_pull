// A service for opening and closing the grippers.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import SetGrippers
// rospy.wait_for_service('set_grippers_service')
// set_grippers = rospy.ServiceProxy('set_grippers_service', SetGrippers)
// set_grippers(False, True) # Close left hand and open right hand.

#include "pr2_pick_manipulation/gripper.h"
#include "pr2_pick_manipulation/SetGrippers.h"
#include "pr2_pick_manipulation/GetGrippers.h"
#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_GRIPPER_SERVICE_H_
#define _PR2_PICK_MANIPULATION_GRIPPER_SERVICE_H_

namespace pr2_pick_manipulation {
class GripperService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer set_grippers_server_;
  ros::ServiceServer get_grippers_server_;
  Gripper left_;
  Gripper right_;
  bool SetGrippersCallback(SetGrippers::Request& request, SetGrippers::Response& response);
  bool GetGrippersCallback(GetGrippers::Request& request, GetGrippers::Response& response);

 public:
  GripperService();
};
};  // namespace pr2_pick_manipulation

#endif
