// A client for raising and lowering the robot's torso.
//
// Sample usage:
//
// #include "pr2_pick_manipulation/torso.h"
// using pr2_pick_manipulation::Torso;
// ... (initialize ros node) ...
// Torso torso;
// torso.SetHeight(0.1);
// torso.SetHeight(Torso::kMaxHeight);
// torso.SetHeight(Torso::kMinHeight);

#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>

#ifndef _PR2_PICK_MANIPULATION_TORSO_H_
#define _PR2_PICK_MANIPULATION_TORSO_H_

namespace pr2_pick_manipulation {
typedef actionlib::SimpleActionClient<
  pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

class Torso {
 private:
  TorsoClient* torso_client_;

 public:
  static const double kMaxHeight = 0.330;
  static const double kMinHeight = 0.015;

  Torso();
  ~Torso();

  // Return true if torso height set successfully, false if not
  // height should be in the range [kMinHeight, kMaxHeight]
  bool SetHeight(double height);
};
};  // namespace pr2_pick_manipulation

#endif // _PR2_PICK_MANIPULATION_TORSO_H_
