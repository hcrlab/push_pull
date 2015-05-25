#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include "pr2_pick_manipulation/torso.h"

using actionlib::SimpleClientGoalState;

namespace pr2_pick_manipulation {

Torso::Torso() {
  torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);

  // wait for the action server to come up
  while(!torso_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the torso action server to come up");
  }
}

Torso::~Torso() {
  delete torso_client_;
}

bool Torso::IsDone() {
	return torso_client_->getState() == SimpleClientGoalState::SUCCEEDED;
}

bool Torso::SetHeight(double height, bool blocking) {
  if (height > Torso::kMaxHeight || height < Torso::kMinHeight) {
    ROS_ERROR("Torso height %0.3f not in allowed range [%0.3f, %0.3f]",
           height, Torso::kMinHeight, Torso::kMaxHeight);
    return false;
   }

  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = height;

  torso_client_->sendGoal(goal);
  ROS_INFO("Sent torso goal %0.3f", height);

	if(blocking) {
		torso_client_->waitForResult();
	}
  

  if (torso_client_->getState() == SimpleClientGoalState::SUCCEEDED) {
    return true;
  }
  else {
    return false;
  }
}
};  // namespace par2_pick_manipulation
