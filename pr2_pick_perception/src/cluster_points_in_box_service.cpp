#include "pr2_pick_perception/cluster_points_in_box_service.h"

#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"
#include "pr2_pick_perception/BoxPoints.h"
#include "pr2_pick_perception/pcl.h"
#include "sensor_msgs/PointCloud2.h"
#include <ros/ros.h>

#include <string>

namespace pr2_pick_perception {
ClusterPointsInBoxService::ClusterPointsInBoxService(const std::string& name)
    : name_(name),
      nh_(),
      server_(
          nh_.advertiseService(name, &ClusterPointsInBoxService::Callback, this)),
      tf_listener_() {}

bool ClusterPointsInBoxService::Callback(BoxPoints::Request& request,
                                       BoxPoints::Response& response) {
  // Get point cloud and remove NaNs.
  ROS_INFO("Waiting for point cloud.");
  boost::shared_ptr<const sensor_msgs::PointCloud2> ros_cloud = 
    boost::make_shared<sensor_msgs::PointCloud2>(request.cluster.pointcloud);
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
  if (!tf_listener_.canTransform(request.frame_id, request.cluster.header.frame_id,
                                 ros::Time(0), &error_msg)) {
    ROS_ERROR("%s", error_msg.c_str());
    return false;
  } else {
    ROS_INFO("Transforming from %s to %s", request.cluster.header.frame_id.c_str(),
             request.frame_id.c_str());
    tf_listener_.lookupTransform(request.frame_id, request.cluster.header.frame_id,
                                 ros::Time(0), cloud_to_request);
  }
  pcl_ros::transformPointCloud(pcl_cloud, request_cloud, cloud_to_request);

  // double box_back = request.center.x + request.dimensions.x / 2.0;
  // double box_front = request.center.x - request.dimensions.x / 2.0;
  // double box_left = request.center.y + request.dimensions.y / 2.0;
  // double box_right = request.center.y - request.dimensions.y / 2.0;
  // double box_top = request.center.z + request.dimensions.z / 2.0;
  // double box_bottom = request.center.z - request.dimensions.z / 2.0;

  for (size_t i = 0; i < request.boxes.size(); i++) {
    response.num_points.push_back(0);
  }
  

  for (size_t i = 0; i < request_cloud.size(); ++i) {
    const pcl::PointXYZRGB& point = request_cloud[i];
    for (size_t j = 0; j < request.boxes.size(); ++j) {
      if (point.x < request.boxes[j].max_x && 
          point.x >= request.boxes[j].min_x && 
          point.y < request.boxes[j].max_y &&
          point.y >= request.boxes[j].min_y && 
          point.z < request.boxes[j].max_z && 
          point.z >= request.boxes[j].min_z) {
        response.num_points[j]++;
      }
    }
  }

  return true;
}
};  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "cluster_points_in_box_service_node");
  pr2_pick_perception::ClusterPointsInBoxService service(
      "perception/get_points_in_box");
  ros::spin();
  return 0;
}

