// A service for driving the robot.
//
// Sample usage (Python):
// from pr2_pick_manipulation.srv import DriveLinear
// rospy.wait_for_service('drive_linear_service')
// drive_linear = rospy.ServiceProxy('drive_linear_service', DriveLinear)
// drive_linear(0, 0.1, 0.2) # Drive left at 0.1 meters / second for 20 cm.
//
// from pr2_pick_manipulation.srv import DriveAngular
// rospy.wait_for_service('drive_angular_service')
// drive_angular = rospy.ServiceProxy('drive_angular_service', DriveAngular)
// # Turn right 90 degrees at 0.1 radians / second
// drive_angular(0.1, DriveAngular.RIGHT_90) 

#include "pr2_pick_manipulation/driver.h"
#include "pr2_pick_manipulation/DriveAngular.h"
#include "pr2_pick_manipulation/DriveLinear.h"
#include <ros/ros.h>

#ifndef _PR2_PICK_MANIPULATION_DRIVER_SERVICE_H_
#define _PR2_PICK_MANIPULATION_DRIVER_SERVICE_H_

namespace pr2_pick_manipulation {
class DriverService {
 private:
  ros::NodeHandle nh_;
  ros::ServiceServer angular_server_;
  ros::ServiceServer linear_server_;
  RobotDriver driver_;
  bool AngularCallback(DriveAngular::Request& request,
                       DriveAngular::Response& response);
  bool LinearCallback(DriveLinear::Request& request,
                      DriveLinear::Response& response);

 public:
  DriverService();
};
};  // namespace pr2_pick_manipulation

#endif
