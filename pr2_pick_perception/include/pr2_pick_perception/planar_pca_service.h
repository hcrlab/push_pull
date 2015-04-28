// A service for moving the robot's arms.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import MoveArm
// move_arm = rospy.ServiceProxy('move_arm_service', MoveArm)
// move_arm.wait_for_service()
// # Move the right arm to the given goal pose.
// move_arm(goal=PoseStamped(...), planning_time=10, group='right_arm',
//   position_tolerance=0, orientation_tolerance=0)

#include <ros/ros.h>
#include "pr2_pick_perception/PlanarPrincipalComponents.h"
#include "pr2_pick_perception/pcl.h"

#ifndef _PR2_PICK_PERCEPTION_PLANAR_PCA_SERVICE_H_
#define _PR2_PICK_PERCEPTION_PLANAR_PCA_SERVICE_H_

namespace pr2_pick_perception {
class PlanarPcaService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  bool Callback(PlanarPrincipalComponents::Request& request,
                PlanarPrincipalComponents::Response& response);

 public:
  PlanarPcaService();
};
};  // namespace pr2_pick_perception

#endif  // _PR2_PICK_PERCEPTION_PLANAR_PCA_SERVICE_H_
