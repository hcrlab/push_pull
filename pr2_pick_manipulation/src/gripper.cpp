#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "pr2_pick_manipulation/gripper.h"

using actionlib::SimpleClientGoalState;

namespace pr2_pick_manipulation {

const std::string Gripper::leftGripperTopic = "l_gripper_controller/gripper_action";
const std::string Gripper::rightGripperTopic = "r_gripper_controller/gripper_action";

Gripper::Gripper(const int gripper_id) : gripper_id_(gripper_id) {
  std::string action_name;
  if (gripper_id == Gripper::LEFT_GRIPPER) {
    action_name = leftGripperTopic;
  } else if (gripper_id == Gripper::RIGHT_GRIPPER) {
    action_name = rightGripperTopic;
  } else {
    ROS_ERROR("Bad gripper ID: %d", gripper_id);
  }
  gripper_client_ = new GripperClient(action_name, true);
  while(!gripper_client_->waitForServer()){
    ROS_INFO("Waiting for the %s action server to come up",
             action_name.c_str());
  }
}

Gripper::~Gripper() {
  delete gripper_client_;
}

bool Gripper::SetPosition(double position, double effort) {
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

double Gripper::GetPosition() {
  tf::StampedTransform transform;
  std::string destination_frame;
  std::string original_frame;

  // set frames based on whether this is a right or left gripper
  if (gripper_id_ == Gripper::LEFT_GRIPPER) {
    destination_frame = "l_gripper_l_finger_tip_link";
    original_frame = "l_gripper_r_finger_tip_link";
  } else if (gripper_id_ == Gripper::RIGHT_GRIPPER) {
    destination_frame = "r_gripper_l_finger_tip_link";
    original_frame = "r_gripper_r_finger_tip_link";
  }
  
  // get the transform between the fingertips
  try {
    transform_listener_.waitForTransform(original_frame, destination_frame, ros::Time(0), ros::Duration(10.0) );
    transform_listener_.lookupTransform(original_frame, destination_frame, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
  }
  
  // subtract small positive offset so that 0 means closed
  double gripper_offset = transform.getOrigin().y();
  ROS_INFO("gripper_offset = %f", gripper_offset);
  return gripper_offset - 0.032;
}

bool Gripper::IsOpen() {
  return GetPosition() > Gripper::OPEN_THRESHOLD;
}

bool Gripper::Open(double effort) {
  return Gripper::SetPosition(Gripper::kOpen, effort);
}

bool Gripper::Close(double effort) {
  return Gripper::SetPosition(Gripper::kClosed, effort);
}
};  // namespace pr2_pick_manipulation
