// A service for tucking the arms.
// Note: you need to run the pr2_tuck_arms_action server.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import TuckArms
// rospy.wait_for_service('tuck_arms_service')
// tuck_arms = rospy.ServiceProxy('tuck_arms_service', TuckArms)
// tuck_arms(false, true) # Tuck the left arm and untuck the right arm.

#include "pr2_common_action_msgs/TuckArmsAction.h"
#include "pr2_pick_manipulation/TuckArms.h"

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_TUCK_ARMS_SERVICE_H_
#define _PR2_PICK_MANIPULATION_TUCK_ARMS_SERVICE_H_

namespace pr2_pick_manipulation {
typedef actionlib::SimpleActionClient<
  pr2_common_action_msgs::TuckArmsAction> TuckArmsClient;

class TuckArmsService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  TuckArmsClient client_;
  bool Callback(TuckArms::Request& request, TuckArms::Response& response);

 public:
  TuckArmsService();
};
};  // namespace pr2_pick_manipulation

#endif
