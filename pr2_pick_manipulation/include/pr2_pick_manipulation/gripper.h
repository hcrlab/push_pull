// A C++ action client for opening and closing the grippers.
//
// Sample usage:
//  pr2_pick_manipulation::Gripper right_gripper(
//    Gripper::RIGHT_GRIPPER);
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
#include <tf/transform_listener.h>
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
  tf::TransformListener transform_listener_;
  const int gripper_id_;
  // Gripper open threshold
  static const double OPEN_THRESHOLD = 0.005;

 public:
  // Canonical "open" position.
  static const double kOpen = 0.09;
  // Canonical "closed" position.
  static const double kClosed = 0.00;

  // Gripper ids
  static const int LEFT_GRIPPER = 0;
  static const int RIGHT_GRIPPER = 1;

  // Gripper action topics
  static const std::string leftGripperTopic;
  static const std::string rightGripperTopic;



  // Constructor that takes the gripper id.
  // @param gripper_id - Gripper::LEFT_GRIPPER or Gripper::RIGHT_GRIPPER
  Gripper(const int gripper_id);

  ~Gripper();

  // Gets the gripper to the given position. .
  // @param position - how wide to open or close the gripper
  // @param effort - now much force to exert, negative is full force
  bool SetPosition(double position, double effort = -1.0);

  // Gets the gripper's current position. Note: may not agree with "SetPosition"
  double GetPosition();

  // Returns whether the gripper is open or not.
  bool IsOpen();

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
