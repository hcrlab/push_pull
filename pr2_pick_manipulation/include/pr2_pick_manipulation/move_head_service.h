// A service for moving the robot's head.
// Note: you need to run the pr2_move_head_action server.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import MoveHead
// rospy.wait_for_service('move_head_service')
// move_head = rospy.ServiceProxy('move_head_service', MoveHead)
// move_head(10, 0, 0, "base_link") # Point the head at a point 10 meters directly in
//                                  # front of the robot in the base_link frame

#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pr2_pick_manipulation/MoveHead.h"

#ifndef _PR2_PICK_MANIPULATION_MOVE_HEAD_SERVICE_H_
#define _PR2_PICK_MANIPULATION_MOVE_HEAD_SERVICE_H_

namespace pr2_pick_manipulation {
typedef actionlib::SimpleActionClient<
  pr2_controllers_msgs::PointHeadAction> MoveHeadClient;

class MoveHeadService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  MoveHeadClient client_;
  bool Callback(MoveHead::Request& request, MoveHead::Response& response);

 public:
  MoveHeadService();
};
};  // namespace pr2_pick_manipulation

#endif // _PR2_PICK_MANIPULATION_MOVE_HEAD_SERVICE_H_
