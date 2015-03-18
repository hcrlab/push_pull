// A C++ action client for opening and closing the grippers.
//
// Sample usage:
//  pr2_pick_manipulation::Gripper right_gripper(
//    "r_gripper_controller/gripper_action");
//  right_gripper.open();
//  right_gripper.close();

#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_GRIPPER_H_
#define _PR2_PICK_MANIPULATION_GRIPPER_H_

namespace pr2_pick_manipulation {
typedef actionlib::SimpleActionClient<
  pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

class Gripper {
 private:
  GripperClient* gripper_client_;  

 public:
  // Constructor that takes the name of the action server to use. For the PR2,
  // this is "r_gripper_controller/gripper_action" or
  // "l_gripper_controller/gripper_action".
  Gripper(const std::string& action_name);

  ~Gripper();

  // Opens the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  bool open();

  // Closes the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  bool close();
};
};  // namespace pr2_pick_manipulation

#endif
