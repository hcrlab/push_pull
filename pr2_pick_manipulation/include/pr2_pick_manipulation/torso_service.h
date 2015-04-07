// A service for moving the torso.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import MoveTorso
// rospy.wait_for_service('torso_service')
// move_torso = rospy.ServiceProxy('torso_service', Torso)
// move_torso(0.1) # Move torso to position 10 cm up.
// move_torso(Torso.MAX_HEIGHT) # Move torso to maximum height.

#include "pr2_pick_manipulation/MoveTorso.h"
#include "pr2_pick_manipulation/torso.h"

#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_TORSO_SERVICE_H_
#define _PR2_PICK_MANIPULATION_TORSO_SERVICE_H_

namespace pr2_pick_manipulation {
class TorsoService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  Torso torso_;
  bool Callback(MoveTorso::Request& request, MoveTorso::Response& response);

 public:
  TorsoService();
};
};  // namespace pr2_pick_manipulation

#endif
