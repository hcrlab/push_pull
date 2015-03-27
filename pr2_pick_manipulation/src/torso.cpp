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

bool Torso::SetHeight(double height) {
  // ## make sure height is in range
  // if (height > Torso::kMaxHeight || height < Torso::kMinHeight) {
  //   printf("Height %f is not in range [0.0, 0.2]\n", height);
  //   return false;
  //  }

  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = height;

  torso_client_->sendGoal(goal);
  printf("Sent goal %f\n", height);

  torso_client_->waitForResult();
  printf("Got result\n");

  if (torso_client_->getState() == SimpleClientGoalState::SUCCEEDED) {
    printf("Success\n");
    return true;
  }
  else {
    printf("Failure\n");
    return false;
  }
}
};  // namespace par2_pick_manipulation
