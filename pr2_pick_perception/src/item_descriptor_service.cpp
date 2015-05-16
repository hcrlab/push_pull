#include "pr2_pick_perception/item_descriptor_service.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pr2_pick_perception/BoundingBox.h"
#include "pr2_pick_perception/ColorHistogram.h"
#include "pr2_pick_perception/GetItemDescriptor.h"
#include "pr2_pick_perception/ItemDescriptor.h"
#include "pr2_pick_perception/pcl.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

#include <string>

namespace pr2_pick_perception {
ItemDescriptorService::ItemDescriptorService(const std::string& name)
    : name_(name),
      nh_(),
      server_(
          nh_.advertiseService(name, &ItemDescriptorService::Callback, this)),
      num_bins_(4) {
  ros::param::param<double>("color_histogram/num_bins", num_bins_, 4);
}

bool ItemDescriptorService::Callback(GetItemDescriptor::Request& request,
                                     GetItemDescriptor::Response& response) {
  const sensor_msgs::PointCloud2& ros_cloud = request.cluster.pointcloud;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_unfiltered;
  pcl::fromROSMsg(ros_cloud, pcl_cloud_unfiltered);
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(pcl_cloud_unfiltered, pcl_cloud, index);

  // Get color histogram.
  std::vector<int> histogram;
  ComputeColorHistogramSeparate(pcl_cloud, num_bins_, &histogram);
  ColorHistogram histogram_msg;
  histogram_msg.num_bins = num_bins_;
  histogram_msg.histogram = histogram;

  // Get planar bounding box.
  geometry_msgs::Pose planar_bbox_midpoint;
  geometry_msgs::Vector3 planar_bbox_dimensions;
  GetPlanarBoundingBox(pcl_cloud, &planar_bbox_midpoint,
                       &planar_bbox_dimensions);

  geometry_msgs::PoseStamped planar_bbox_midpoint_stamped;
  planar_bbox_midpoint_stamped.header.frame_id =
      request.cluster.header.frame_id;
  planar_bbox_midpoint_stamped.pose = planar_bbox_midpoint;

  pr2_pick_perception::BoundingBox bounding_box;
  bounding_box.pose = planar_bbox_midpoint_stamped;
  bounding_box.dimensions = planar_bbox_dimensions;

  // Return descriptor.
  ItemDescriptor descriptor;
  descriptor.histogram = histogram_msg;
  descriptor.planar_bounding_box = bounding_box;

  response.descriptor = descriptor;
  return true;
}
};  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "item_descriptor_service_node");
  pr2_pick_perception::ItemDescriptorService service(
      "perception/get_item_descriptor");
  ros::spin();
  return 0;
}
