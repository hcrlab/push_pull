#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <ros/ros.h>

#include "pr2_pick_manipulation/gripper.h"

using actionlib::SimpleClientGoalState;

namespace pr2_pick_manipulation {
Gripper::Gripper(const std::string& action_name) {
  gripper_client_ = new GripperClient(action_name, true);
  while(!gripper_client_->waitForServer()){
    ROS_INFO("Waiting for the %s action server to come up",
             action_name.c_str());
  }
}

Gripper::~Gripper() {
  delete gripper_client_;
}

bool Gripper::open() {
  pr2_controllers_msgs::Pr2GripperCommandGoal open;
  open.command.position = 0.08;
  open.command.max_effort = -1.0;  // Do not limit effort (negative)
  
  gripper_client_->sendGoal(open);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}

bool Gripper::close() {
  pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
  squeeze.command.position = 0.0;
  squeeze.command.max_effort = -1.0;  

  gripper_client_->sendGoal(squeeze);
  gripper_client_->waitForResult();
  if(gripper_client_->getState() == SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    return false;
  }
}
};  // namespace pr2_pick_manipulation
