#ifndef _PR2_PICK_PERCEPTION_ITEM_SEGMENTATION_H_
#define _PR2_PICK_PERCEPTION_ITEM_SEGMENTATION_H_

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pr2_pick_perception/SegmentItems.h"
#include "ros/ros.h"

#include <string>
#include <vector>

namespace pr2_pick_perception {
class ItemSegmentationService {
 public:
  ItemSegmentationService(const std::string& name);

 private:
  std::string name_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::ServiceServer server_;
  int min_cluster_points_;
  int max_cluster_points_;
  bool Callback(SegmentItems::Request& request,
                SegmentItems::Response& response);
};
}  // namespace pr2_pick_perception

#endif
