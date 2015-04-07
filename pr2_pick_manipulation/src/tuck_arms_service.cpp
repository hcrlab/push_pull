#include "pr2_common_action_msgs/TuckArmsAction.h"
#include "pr2_pick_manipulation/TuckArms.h"
#include "pr2_pick_manipulation/tuck_arms_service.h"

#include <actionlib/client/simple_client_goal_state.h>
#include <ros/ros.h>

using actionlib::SimpleClientGoalState;
using pr2_common_action_msgs::TuckArmsGoal;

namespace pr2_pick_manipulation {
TuckArmsService::TuckArmsService()
    : nh_(),
      server_(nh_.advertiseService("tuck_arms_service",
                                   &TuckArmsService::Callback,
                                   this)),
      client_("tuck_arms", true) {
}

bool TuckArmsService::Callback(TuckArms::Request& request,
                               TuckArms::Response& response) {
  ROS_INFO("Waiting for tuck arms action server...");
  client_.waitForServer();
  ROS_INFO("Tuck arms service ready.");
  TuckArmsGoal goal;
  goal.tuck_left = request.tuck_left;
  goal.tuck_right = request.tuck_right;
  client_.sendGoal(goal);
  bool success = false;
  success = client_.waitForResult(ros::Duration(30.0));
  if (success) {
    SimpleClientGoalState state = client_.getState();
    if (state != SimpleClientGoalState::SUCCEEDED) {
      ROS_WARN("Tuck arms action ended in state: %s", state.toString().c_str());
    }
    response.success = true;
    return true;
  } else {
    response.success = false;
    return false;
  }
}

};  // namespace pr2_pick_manipulation
