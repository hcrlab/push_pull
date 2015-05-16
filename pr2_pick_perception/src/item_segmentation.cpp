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
      nh_(name),
      nh_local_("~"),
      min_cluster_points_(150),
      max_cluster_points_(100000),
      server_(nh_.advertiseService(name, &ItemSegmentationService::Callback,
                                   this)) {
  nh_local_.param("min_cluster_points", min_cluster_points_, 150);
  nh_local_.param("max_cluster_points", max_cluster_points_, 100000);
}

bool ItemSegmentationService::Callback(SegmentItems::Request& request,
                                       SegmentItems::Response& response) {
  sensor_msgs::PointCloud2& cell_pc_ros = request.cloud;
  PointCloud<PointXYZRGB>::Ptr cell_pc(new PointCloud<PointXYZRGB>());
  pcl::fromROSMsg(*cell_pc, cell_pc_ros);

  std::vector<PointCloud<PointXYZRGB>::Ptr> clusters;

  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>);
  tree->setInputCloud(cell_pc);
  std::vector<pcl::PointIndices> clustersInd;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(min_cluster_points_);
  ec.setMaxClusterSize(max_cluster_points_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cell_pc);
  ec.extract(clustersInd);

  for (int i = 0; i < clustersInd.size(); ++i) {
    PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>);
    PointXYZRGB point;
    for (int j = 0; j < clustersInd[i].indices.size(); j++) {
      int index = clustersInd[i].indices[j];
      point.x = cell_pc->points[index].x;
      point.y = cell_pc->points[index].y;
      point.z = cell_pc->points[index].z;
      point.r = cell_pc->points[index].r;
      point.g = cell_pc->points[index].g;
      point.b = cell_pc->points[index].b;
      cluster->points.push_back(point);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    clusters.push_back(cluster);
  }

  // copy the clusters to the object ObjectList
  pr2_pick_perception::ClusterList clusterlist;
  for (int i = 0; i < clusters.size(); i++) {
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = bin_frame_id_;
    cluster.header.stamp = pc_timestamp_;
    std::stringstream ss;
    ss << "cluster_" << i;
    cluster.id = ss.str();
    pcl::toROSMsg(*clusters[i], cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }
  if (clusters.size() == 0) {
    ROS_INFO("No clusters found. Returning whole cropped PC");
    pr2_pick_perception::Cluster cluster;
    cluster.header.frame_id = bin_frame_id_;
    cluster.header.stamp = pc_timestamp_;
    std::stringstream ss;
    ss << "cluster_0";
    cluster.id = ss.str();
    pcl::toROSMsg(*cell_pc, cluster.pointcloud);

    clusterlist.clusters.push_back(cluster);
  }
  response.locations = clusterlist;
}
};
