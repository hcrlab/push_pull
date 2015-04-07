// A C++ action client for opening and closing the grippers.
//
// Sample usage:
//  pr2_pick_manipulation::Gripper right_gripper(
//    "r_gripper_controller/gripper_action");
//  right_gripper.open();
//  right_gripper.close();
//
// Note: The gripper does not work quite right in Gazebo

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
  // Canonical "open" position. The gripper may be able to open as wide is 0.087,
  // but we are limiting it to 0.08 for this client.
  static const double kOpen = 0.08;
  // Canonical "closed" position.
  static const double kClosed = 0.00;

  // Constructor that takes the name of the action server to use. For the PR2,
  // this is "r_gripper_controller/gripper_action" or
  // "l_gripper_controller/gripper_action".
  Gripper(const std::string& action_name);

  ~Gripper();

  // Sets the gripper to the given position. Returns true if successful, false
  // otherwise.
  // @param position - how wide to open or close the gripper
  // @param effort - now much force to exert, negative is full force
  bool setPosition(double position, double effort = -1.0);

  // Opens the gripper. Returns true if the gripper opened successfully, false
  // otherwise. Uses default effort.
  bool open();

  // Closes the gripper. Returns true if the gripper opened successfully, false
  // otherwise. Uses default effort.
  bool close();
};
};  // namespace pr2_pick_manipulation

#endif
