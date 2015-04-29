// A service for getting a descriptor of an item.
#include "pr2_pick_perception/GetItemDescriptor.h"
#include "pr2_pick_perception/pcl.h"
#include <ros/ros.h>

#include <string>

#ifndef _PR2_PICK_PERCEPTION_ITEM_DESCRIPTOR_SERVICE_H_
#define _PR2_PICK_PERCEPTION_ITEM_DESCRIPTOR_SERVICE_H_

namespace pr2_pick_perception {
class ItemDescriptorService {
 private:
  std::string name_;
  ros::NodeHandle nh_;
  ros::ServiceServer server_;
  bool Callback(GetItemDescriptor::Request& request,
                GetItemDescriptor::Response& response);

 public:
  ItemDescriptorService(const std::string& name);
};
};  // namespace pr2_pick_perception

#endif  // _PR2_PICK_PERCEPTION_ITEM_DESCRIPTOR_SERVICE_H_
