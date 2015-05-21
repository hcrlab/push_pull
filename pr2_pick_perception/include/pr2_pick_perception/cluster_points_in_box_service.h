// A service for getting a descriptor of an item.
#include "pr2_pick_perception/BoxPoints.h"
#include "pr2_pick_perception/pcl.h"
#include "ros/ros.h"
#include "tf/transform_listener.h"

#include <string>

#ifndef _PR2_PICK_PERCEPTION_CLUSTER_POINTS_IN_BOX_SERVICE_H_
#define _PR2_PICK_PERCEPTION_CLUSTER_POINTS_IN_BOX_SERVICE_H_

namespace pr2_pick_perception {
class ClusterPointsInBoxService {
 private:
  std::string name_;
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  tf::TransformListener tf_listener_;

  bool Callback(BoxPoints::Request& request,
                BoxPoints::Response& response);

 public:
  ClusterPointsInBoxService(const std::string& name);
};
};  // namespace pr2_pick_perception

#endif  // _PR2_PICK_PERCEPTION_CLUSTER_POINTS_IN_BOX_SERVICE_H_
