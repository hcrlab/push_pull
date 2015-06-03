#include "pr2_pick_perception/item_segmentation.h"

#include "pcl/filters/passthrough.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pr2_pick_perception/Cluster.h"
#include "pr2_pick_perception/ClusterList.h"
#include "pr2_pick_perception/pcl.h"
#include "ros/ros.h"

#include <string>
#include <vector>

using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace pr2_pick_perception {
ItemSegmentationService::ItemSegmentationService(const std::string& name)
    : name_(name),
      nh_(),
      nh_local_("~"),
      server_(
          nh_.advertiseService(name, &ItemSegmentationService::Callback, this)),
      min_cluster_points_(150),
      max_cluster_points_(100000) {
  nh_local_.param("min_cluster_points", min_cluster_points_, 150);
  nh_local_.param("max_cluster_points", max_cluster_points_, 100000);
}

bool ItemSegmentationService::Callback(SegmentItems::Request& request,
                                       SegmentItems::Response& response) {
  // Get input cloud.
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  std::string cloud_frame_id = request.cloud.header.frame_id;
  ros::Time cloud_stamp = request.cloud.header.stamp;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(cell_pc_ros, *cell_pc);

  PointCloud<PointXYZRGB>::Ptr cell_pc_filtered(new PointCloud<PointXYZRGB>());

  // Remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
  outlier_removal.setInputCloud(cell_pc);
  int mean_k;
  if (!ros::param::get("mean_k", mean_k)) {
    mean_k = 50;
  }
  outlier_removal.setMeanK(mean_k);
  double stddev;
  if (!ros::param::get("stddev", stddev)) {
    stddev = 1;
  }
  outlier_removal.setStddevMulThresh(stddev);
  outlier_removal.filter(*cell_pc_filtered);

  // Remove NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cell_pc_filtered, *cell_pc_filtered, indices);

  // Do clustering.
  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;
  ClusterWithEuclidean(*cell_pc_filtered, 0.0254, min_cluster_points_,
                       max_cluster_points_, &clusters);
  // ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
  if (clusters.size() != request.items.size()) {
    ROS_INFO("Euclidean clustering yielded %ld items, expected %ld",
             clusters.size(), request.items.size());
    ClusterBinItems(*cell_pc_filtered, request.items.size(), &clusters);
  }

  // Copy the clusters back to the response.
  pr2_pick_perception::ClusterList& clusterlist = response.clusters;
  for (size_t i = 0; i < clusters.size(); i++) {
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_" << i;
    cluster.id = ss.str();
    pcl::toROSMsg(*clusters[i], cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  if (clusters.size() == 0) {
    ROS_INFO("No clusters found. Returning whole cropped PC");
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_0";
    cluster.id = ss.str();
    pcl::toROSMsg(*cell_pc_filtered, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  return true;
}

ItemOverSegmentationService::ItemOverSegmentationService(
    const std::string& name)
    : name_(name),
      nh_(),
      nh_local_("~"),
      server_(nh_.advertiseService(name, &ItemOverSegmentationService::Callback,
                                   this)),
      min_cluster_points_(150),
      max_cluster_points_(100000) {
  nh_local_.param("min_cluster_points", min_cluster_points_, 150);
  nh_local_.param("max_cluster_points", max_cluster_points_, 100000);
}

bool ItemOverSegmentationService::Callback(SegmentItems::Request& request,
                                           SegmentItems::Response& response) {
  // Get input cloud.
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  std::string cloud_frame_id = request.cloud.header.frame_id;
  ros::Time cloud_stamp = request.cloud.header.stamp;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(cell_pc_ros, *cell_pc);

  PointCloud<PointXYZRGB>::Ptr cell_pc_filtered(new PointCloud<PointXYZRGB>());

  // Remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
  outlier_removal.setInputCloud(cell_pc);
  int mean_k;
  if (!ros::param::get("mean_k", mean_k)) {
    mean_k = 50;
  }
  outlier_removal.setMeanK(mean_k);
  double stddev;
  if (!ros::param::get("stddev", stddev)) {
    stddev = 1;
  }
  outlier_removal.setStddevMulThresh(stddev);
  outlier_removal.filter(*cell_pc_filtered);

  // Remove NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cell_pc_filtered, *cell_pc_filtered, indices);

  // Do clustering.
  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;
  ClusterWithEuclidean(*cell_pc_filtered, 0.0254, min_cluster_points_,
                       max_cluster_points_, &clusters);
  // ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
  if (clusters.size() != request.items.size()) {
    ROS_INFO("Euclidean clustering yielded %ld items, expected %ld",
             clusters.size(), request.items.size());
    ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
    // ClusterBinItems(*cell_pc_filtered, request.items.size(), &clusters);
  }

  // Copy the clusters back to the response.
  pr2_pick_perception::ClusterList& clusterlist = response.clusters;
  for (size_t i = 0; i < clusters.size(); i++) {
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_" << i;
    cluster.id = ss.str();
    pcl::toROSMsg(*clusters[i], cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  if (clusters.size() == 0) {
    ROS_INFO("No clusters found. Returning whole cropped PC");
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_0";
    cluster.id = ss.str();
    pcl::toROSMsg(*cell_pc_filtered, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  return true;
}

ItemKMeansSegmentationService::ItemKMeansSegmentationService(
    const std::string& name)
    : name_(name),
      nh_(),
      nh_local_("~"),
      server_(nh_.advertiseService(
          name, &ItemKMeansSegmentationService::Callback, this)),
      min_cluster_points_(150),
      max_cluster_points_(100000) {
  nh_local_.param("min_cluster_points", min_cluster_points_, 150);
  nh_local_.param("max_cluster_points", max_cluster_points_, 100000);
}

bool ItemKMeansSegmentationService::Callback(SegmentItems::Request& request,
                                             SegmentItems::Response& response) {
  // Get input cloud.
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  std::string cloud_frame_id = request.cloud.header.frame_id;
  ros::Time cloud_stamp = request.cloud.header.stamp;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(cell_pc_ros, *cell_pc);

  PointCloud<PointXYZRGB>::Ptr cell_pc_filtered(new PointCloud<PointXYZRGB>());

  // Remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
  outlier_removal.setInputCloud(cell_pc);
  int mean_k;
  if (!ros::param::get("mean_k", mean_k)) {
    mean_k = 50;
  }
  outlier_removal.setMeanK(mean_k);
  double stddev;
  if (!ros::param::get("stddev", stddev)) {
    stddev = 1;
  }
  outlier_removal.setStddevMulThresh(stddev);
  outlier_removal.filter(*cell_pc_filtered);

  // Remove NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cell_pc_filtered, *cell_pc_filtered, indices);

  // Do clustering.
  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;
  ClusterWithEuclidean(*cell_pc_filtered, 0.0254, min_cluster_points_,
                       max_cluster_points_, &clusters);
  // ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
  if (clusters.size() != request.items.size()) {
    ROS_INFO("Euclidean clustering yielded %ld items, expected %ld",
             clusters.size(), request.items.size());
    // ClusterBinItems(*cell_pc_filtered, request.items.size(), &clusters);
    ClusterWithKMeans(*cell_pc_filtered, request.items.size(), &clusters);
  }

  // Copy the clusters back to the response.
  pr2_pick_perception::ClusterList& clusterlist = response.clusters;
  for (size_t i = 0; i < clusters.size(); i++) {
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_" << i;
    cluster.id = ss.str();
    pcl::toROSMsg(*clusters[i], cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  if (clusters.size() == 0) {
    ROS_INFO("No clusters found. Returning whole cropped PC");
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_0";
    cluster.id = ss.str();
    pcl::toROSMsg(*cell_pc_filtered, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  return true;
}

ItemOverKMeansSegmentationService::ItemOverKMeansSegmentationService(
    const std::string& name)
    : name_(name),
      nh_(),
      nh_local_("~"),
      server_(nh_.advertiseService(
          name, &ItemOverKMeansSegmentationService::Callback, this)),
      min_cluster_points_(150),
      max_cluster_points_(100000) {
  nh_local_.param("min_cluster_points", min_cluster_points_, 150);
  nh_local_.param("max_cluster_points", max_cluster_points_, 100000);
}

bool ItemOverKMeansSegmentationService::Callback(
    SegmentItems::Request& request, SegmentItems::Response& response) {
  // Get input cloud.
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  std::string cloud_frame_id = request.cloud.header.frame_id;
  ros::Time cloud_stamp = request.cloud.header.stamp;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(cell_pc_ros, *cell_pc);

  PointCloud<PointXYZRGB>::Ptr cell_pc_filtered(new PointCloud<PointXYZRGB>());

  // Remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_removal;
  outlier_removal.setInputCloud(cell_pc);
  int mean_k;
  if (!ros::param::get("mean_k", mean_k)) {
    mean_k = 50;
  }
  outlier_removal.setMeanK(mean_k);
  double stddev;
  if (!ros::param::get("stddev", stddev)) {
    stddev = 1;
  }
  outlier_removal.setStddevMulThresh(stddev);
  outlier_removal.filter(*cell_pc_filtered);

  // Remove NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cell_pc_filtered, *cell_pc_filtered, indices);

  // Do clustering.
  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;
  ClusterWithEuclidean(*cell_pc_filtered, 0.0254, min_cluster_points_,
                       max_cluster_points_, &clusters);
  // ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
  if (clusters.size() != request.items.size()) {
    ROS_INFO("Euclidean clustering yielded %ld items, expected %ld",
             clusters.size(), request.items.size());
    // ClusterWithRegionGrowing(*cell_pc_filtered, &clusters);
    ClusterBinItemsWithKMeans(*cell_pc_filtered, request.items.size(),
                              &clusters);
  }

  // Copy the clusters back to the response.
  pr2_pick_perception::ClusterList& clusterlist = response.clusters;
  for (size_t i = 0; i < clusters.size(); i++) {
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_" << i;
    cluster.id = ss.str();
    pcl::toROSMsg(*clusters[i], cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  if (clusters.size() == 0) {
    ROS_INFO("No clusters found. Returning whole cropped PC");
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = cloud_frame_id;
    cluster.header.stamp = cloud_stamp;
    std::stringstream ss;
    ss << "cluster_0";
    cluster.id = ss.str();
    pcl::toROSMsg(*cell_pc_filtered, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  return true;
}

}  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "item_segmentation");
  pr2_pick_perception::ItemSegmentationService service("segment_items");
  pr2_pick_perception::ItemOverSegmentationService over_segmentation_service(
      "over_segment_items");
  pr2_pick_perception::ItemKMeansSegmentationService
      kmeans_segmentation_service("kmeans_segment_items");
  pr2_pick_perception::ItemOverKMeansSegmentationService
      over_kmeans_segmentation_service("over_kmeans_segment_items");
  ros::spin();
  return 0;
}
