// A service for getting a descriptor of an item.
#include "pr2_pick_perception/CountPointsInBox.h"
#include "pr2_pick_perception/pcl.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include <string>

#ifndef _PR2_PICK_PERCEPTION_COUNT_POINTS_IN_BOX_SERVICE_H_
#define _PR2_PICK_PERCEPTION_COUNT_POINTS_IN_BOX_SERVICE_H_

namespace pr2_pick_perception {
class CountPointsInBoxService {
 private:
  std::string name_;
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  std::string pc_topic_;
  tf::TransformListener tf_listener_;

  bool Callback(CountPointsInBox::Request& request,
                CountPointsInBox::Response& response);

 public:
  CountPointsInBoxService(const std::string& name, const std::string& pc_topic);
};
};  // namespace pr2_pick_perception

#endif  // _PR2_PICK_PERCEPTION_COUNT_POINTS_IN_BOX_SERVICE_H_
