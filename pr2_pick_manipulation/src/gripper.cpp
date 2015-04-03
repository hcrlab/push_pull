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

bool Gripper::setPosition(double position, double effort) {
  if (position > Gripper::kOpen || position < Gripper::kClosed) {
    ROS_ERROR("Gripper position %0.3f not in allowed range [%0.3f, %0.3f]",
              position, Gripper::kOpen, Gripper::kClosed);
    return false;
  }

  pr2_controllers_msgs::Pr2GripperCommandGoal goal;
  goal.command.position = position;
  goal.command.max_effort = effort;

  gripper_client_->sendGoal(goal);
  gripper_client_->waitForResult();
  SimpleClientGoalState state = gripper_client_->getState();
  if(state == SimpleClientGoalState::SUCCEEDED) {
    return true;
  } else {
    ROS_ERROR("Gripper goal state: %s\n", state.toString().c_str());
    return false;
  }
}

bool Gripper::open() {
  return Gripper::setPosition(0.08, -1.0);
}

bool Gripper::close() {
  return Gripper::setPosition(0.00, -1.0);
}
};  // namespace pr2_pick_manipulation
