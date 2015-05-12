#include "pr2_pick_perception/count_points_in_box_service.h"

#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "pr2_pick_perception/CountPointsInBox.h"
#include "pr2_pick_perception/pcl.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

#include <string>

namespace pr2_pick_perception {
CountPointsInBoxService::CountPointsInBoxService(const std::string& name,
                                                 const std::string& pc_topic)
    : name_(name),
      nh_(),
      server_(
          nh_.advertiseService(name, &CountPointsInBoxService::Callback, this)),
      pc_topic_(pc_topic),
      tf_listener_() {}

bool CountPointsInBoxService::Callback(CountPointsInBox::Request& request,
                                       CountPointsInBox::Response& response) {
  // Get point cloud and remove NaNs.
  ROS_INFO("Waiting for point cloud.");
  boost::shared_ptr<const sensor_msgs::PointCloud2> ros_cloud =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc_topic_);
  ROS_INFO("Got point cloud.");
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_unfiltered;
  pcl::fromROSMsg(*ros_cloud, pcl_cloud_unfiltered);
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(pcl_cloud_unfiltered, pcl_cloud, index);

  // Transform point cloud to request frame.
  pcl::PointCloud<pcl::PointXYZRGB> request_cloud;
  std::string error_msg;
  tf::StampedTransform cloud_to_request;
  if (!tf_listener_.canTransform("torso_lift_link", ros_cloud->header.frame_id,
                                 ros::Time(0), &error_msg)) {
    ROS_ERROR("%s", error_msg.c_str());
    return false;
  } else {
    ROS_INFO("Transforming from %s to %s", ros_cloud->header.frame_id.c_str(),
             request.frame_id.c_str());
    tf_listener_.lookupTransform(request.frame_id, ros_cloud->header.frame_id,
                                 ros::Time(0), cloud_to_request);
  }
  pcl_ros::transformPointCloud(pcl_cloud, request_cloud, cloud_to_request);

  double box_back = request.center.x + request.dimensions.x / 2.0;
  double box_front = request.center.x - request.dimensions.x / 2.0;
  double box_left = request.center.y + request.dimensions.y / 2.0;
  double box_right = request.center.y - request.dimensions.y / 2.0;
  double box_top = request.center.z + request.dimensions.z / 2.0;
  double box_bottom = request.center.z - request.dimensions.z / 2.0;

  for (size_t i = 0; i < request_cloud.size(); ++i) {
    const pcl::PointXYZRGB& point = request_cloud[i];
    if (point.x < box_back && point.x >= box_front && point.y < box_left &&
        point.y >= box_right && point.z < box_top && point.z >= box_bottom) {
      response.num_points++;
    }
  }

  return true;
}
};  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "count_points_in_box_service_node");
  pr2_pick_perception::CountPointsInBoxService service(
      "perception/count_points_in_box",
      "/head_mount_kinect/depth_registered/points");
  ros::spin();
  return 0;
}
