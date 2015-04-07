// A C++ action client for opening and closing the grippers.
//
// Sample usage:
//  pr2_pick_manipulation::Gripper right_gripper(
//    "r_gripper_controller/gripper_action");
//  right_gripper.Open();
//  right_gripper.Close();
//
// Notes:
// 1. The gripper does not work quite right in Gazebo
// 2. Return value is based on whether the gripper reached the target position.
//    So, if the gripper is closing around an object, Close() will typically
//    return false because the object stalls the gripper before it reaches
//    the goal position.

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
  // Canonical "open" position.
  static const double kOpen = 0.09;
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
  bool SetPosition(double position, double effort = -1.0);

  // Opens the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // @param effort - defaults to -1.0, to open with unlimited effort
  bool Open(double effort = -1.0);

  // Closes the gripper. Returns true if the gripper opened successfully, false
  // otherwise.
  // @param effort - defaults to 50.0 to close gently
  bool Close(double effort = -1.0);
};
};  // namespace pr2_pick_manipulation

#endif
