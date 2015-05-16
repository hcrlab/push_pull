#include "pr2_pick_perception/item_segmentation.h"

#include "pcl/filters/passthrough.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "pr2_pick_perception/Cluster.h"
#include "pr2_pick_perception/ClusterList.h"

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

void ItemSegmentationService::ClusterWithEuclidean(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* clusters) {
  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>);
  tree->setInputCloud(cloud.makeShared());
  std::vector<pcl::PointIndices> clustersInd;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(min_cluster_points_);
  ec.setMaxClusterSize(max_cluster_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud.makeShared());
  ec.extract(clustersInd);

  for (size_t i = 0; i < clustersInd.size(); ++i) {
    PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
    PointXYZRGB point;
    for (size_t j = 0; j < clustersInd[i].indices.size(); j++) {
      int index = clustersInd[i].indices[j];
      const PointXYZRGB& point = cloud[index];
      cluster->push_back(point);
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters->push_back(cluster);
  }
}

bool ItemSegmentationService::Callback(SegmentItems::Request& request,
                                       SegmentItems::Response& response) {
  // Get input cloud.
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  std::string cloud_frame_id = request.cloud.header.frame_id;
  ros::Time cloud_stamp = request.cloud.header.stamp;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(cell_pc_ros, *cell_pc);

  // Do clustering.
  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;
  ClusterWithEuclidean(*cell_pc, &clusters);

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
    pcl::toROSMsg(*cell_pc, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }

  return true;
}
}  // namespace pr2_pick_perception

int main(int argc, char** argv) {
  ros::init(argc, argv, "item_segmentation");
  pr2_pick_perception::ItemSegmentationService service("segment_items");
  ros::spin();
  return 0;
}
