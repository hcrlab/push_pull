#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#include "pr2_pick_manipulation/torso.h"

using actionlib::SimpleClientGoalState;

namespace pr2_pick_manipulation {

Torso::Torso() {
  torso_client_ =
      new TorsoClient("torso_controller/position_joint_action", true);

  // wait for the action server to come up
  while (!torso_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the torso action server to come up");
  }
}

Torso::~Torso() { delete torso_client_; }

bool Torso::IsDone() {
  return torso_client_->getState() == SimpleClientGoalState::SUCCEEDED;
}

bool Torso::SetHeight(double height, bool blocking) {
  if (height > Torso::kMaxHeight || height < Torso::kMinHeight) {
    ROS_ERROR("Torso height %0.3f not in allowed range [%0.3f, %0.3f]", height,
              Torso::kMinHeight, Torso::kMaxHeight);
    return false;
  }

  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = height;
  ROS_INFO("About to send torso goal!");
  torso_client_->sendGoal(goal);
  ROS_INFO("Sent torso goal %0.3f", height);

  if (blocking) {
    torso_client_->waitForResult(ros::Duration(60));
    ROS_INFO("Got torso client result");
  }

  if (torso_client_->getState() == SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Torso goal success");
    return true;
  } else {
    ROS_INFO("Torso goal failed");
    return false;
  }
}
};  // namespace par2_pick_manipulation
