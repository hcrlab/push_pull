#include "pr2_pick_perception/pcl.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/distances.h"
#include "pcl/common/pca.h"
#include "pcl/filters/filter.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"
#include "pcl/segmentation/conditional_euclidean_clustering.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/region_growing_rgb.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include <Eigen/Dense>

#include <algorithm>
#include <math.h>
#include <vector>

using geometry_msgs::PoseStamped;
using geometry_msgs::Quaternion;
using geometry_msgs::Vector3;
using pcl::PointCloud;
using pcl::PointXYZRGB;
using std::vector;

namespace pr2_pick_perception {
void PlanarPrincipalComponents(const PointCloud<PointXYZRGB>& cloud,
                               Quaternion* component1, Quaternion* component2,
                               double* value1, double* value2) {
  PointCloud<PointXYZRGB>::Ptr projected(new PointCloud<PointXYZRGB>(cloud));

  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(cloud, centroid);

  // Project points onto XY plane.
  for (size_t i = 0; i < projected->points.size(); ++i) {
    PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }
  for (size_t i = 0; i < projected->points.size(); ++i) {
    PointXYZRGB& point = projected->at(i);
    point.x -= centroid(0);
    point.y -= centroid(1);
  }
  pcl::PCA<PointXYZRGB> pca(true);
  pca.setInputCloud(projected);

  // Return eigenvalues.
  Eigen::Vector3f values = pca.getEigenValues();
  *value1 = values(0);
  *value2 = values(1);

  // Return eigenvectors.
  Eigen::Matrix3f vectors = pca.getEigenVectors();
  double theta0 = atan2(vectors(1, 0), vectors(0, 0));
  double theta1 = atan2(vectors(1, 1), vectors(0, 1));

  Eigen::Quaternion<double> q1;
  q1 = Eigen::AngleAxis<double>(theta0, Eigen::Vector3d::UnitZ());
  component1->w = q1.w();
  component1->x = q1.x();
  component1->y = q1.y();
  component1->z = q1.z();

  Eigen::Quaternion<double> q2;
  q2 = Eigen::AngleAxis<double>(theta1, Eigen::Vector3d::UnitZ());
  component2->w = q2.w();
  component2->x = q2.x();
  component2->y = q2.y();
  component2->z = q2.z();
}

void GetBoundingBox(const sensor_msgs::PointCloud2& cloud,
                    geometry_msgs::TransformStamped* transform,
                    geometry_msgs::Point* bbox) {
  // Get principal components
  // geometry_msgs::Quaternion component1, component2;
  // double value1, value2;
  // PlanarPrincipalComponents(cloud, &component1, &component2, &value1,
  // &value2);

  // Build a provisional frame. Axes in the right orientation, but we don't know
  // the origin yet. Use centroid.
  // tf::Transform provisional_frame = tf::Transform(orientation, origin);

  // for each point:
  //   transform it into the provisional frame
  //   cumulatively track greatest and smallest x, y, z

  // Build the item frame
  // origin at smallest x, y, z
  // orientation same as provisional frame

  // construct the bbox point
  // largest - smallest x, y, z
}

void GetPlanarBoundingBox(const PointCloud<PointXYZRGB>& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions) {
  // Project points onto XY plane.
  PointCloud<PointXYZRGB>::Ptr projected(new PointCloud<PointXYZRGB>(cloud));
  for (size_t i = 0; i < projected->points.size(); ++i) {
    PointXYZRGB& point = projected->at(i);
    point.z = 0;
  }

  // Compute PCA.
  pcl::PCA<PointXYZRGB> pca(true);
  pca.setInputCloud(projected);

  // Get eigenvectors.
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
  // Because we projected points on the XY plane, we add in the Z vector as the
  // 3rd eigenvector.
  eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
  Eigen::Quaternionf q1(eigenvectors);

  // Find min/max x and y, based on the points in eigenspace.
  PointCloud<PointXYZRGB>::Ptr eigen_projected(
      new PointCloud<PointXYZRGB>(cloud));
  pca.project(cloud, *eigen_projected);

  pcl::PointXYZRGB eigen_min;
  pcl::PointXYZRGB eigen_max;
  pcl::getMinMax3D(*eigen_projected, eigen_min, eigen_max);
  double x_length = eigen_max.x - eigen_min.x;
  double y_length = eigen_max.y - eigen_min.y;

  // The points in eigenspace all have z values of 0. Get min/max z from the
  // original point cloud data.
  pcl::PointXYZRGB cloud_min;
  pcl::PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  double z_length = cloud_max.z - cloud_min.z;

  // Compute midpoint, defined as the midpoint between the minimum and maximum
  // points, in x, y, and z directions. The centroid is an average that depends
  // on the density of points, which doesn't give you the geometric center of
  // the point cloud.
  PointXYZRGB eigen_center;
  eigen_center.x = eigen_min.x + x_length / 2;
  eigen_center.y = eigen_min.y + y_length / 2;
  eigen_center.z = 0;
  PointXYZRGB center;
  pca.reconstruct(eigen_center, center);
  center.z = z_length / 2 + cloud_min.z;

  // Output midpoint.
  midpoint->position.x = center.x;
  midpoint->position.y = center.y;
  midpoint->position.z = center.z;
  midpoint->orientation.w = q1.w();
  midpoint->orientation.x = q1.x();
  midpoint->orientation.y = q1.y();
  midpoint->orientation.z = q1.z();

  // Output dimensions.
  dimensions->x = x_length;
  dimensions->y = y_length;
  dimensions->z = z_length;
}

void ComputeColorHistogramSeparate(const PointCloud<PointXYZRGB>& cloud,
                                   const int num_bins,
                                   std::vector<int>* histogram) {
  double bin_size = 255.0 / num_bins;
  histogram->clear();
  histogram->resize(3 * num_bins);
  for (size_t i = 0; i < histogram->size(); ++i) {
    (*histogram)[i] = 0;
  }

  for (size_t i = 0; i < cloud.size(); ++i) {
    const PointXYZRGB& point = cloud[i];
    uint8_t red = point.r;
    uint8_t green = point.g;
    uint8_t blue = point.b;

    int red_bin =
        std::min(static_cast<int>(floor(red / bin_size)), num_bins - 1);
    int green_bin =
        std::min(static_cast<int>(floor(green / bin_size)), num_bins - 1);
    int blue_bin =
        std::min(static_cast<int>(floor(blue / bin_size)), num_bins - 1);
    (*histogram)[red_bin] += 1;
    (*histogram)[num_bins + green_bin] += 1;
    (*histogram)[num_bins + num_bins + blue_bin] += 1;
  }
}

void ComputeColorHistogram(const PointCloud<PointXYZRGB>& cloud,
                           const int num_bins, std::vector<int>* histogram) {
  double bin_size = 255.0 / num_bins;
  histogram->clear();
  histogram->resize(num_bins * num_bins * num_bins);
  for (size_t i = 0; i < histogram->size(); ++i) {
    (*histogram)[i] = 0;
  }

  for (size_t i = 0; i < cloud.size(); ++i) {
    const PointXYZRGB& point = cloud[i];
    uint8_t red = point.r;
    uint8_t green = point.g;
    uint8_t blue = point.b;

    int red_bin =
        std::min(static_cast<int>(floor(red / bin_size)), num_bins - 1);
    int green_bin =
        std::min(static_cast<int>(floor(green / bin_size)), num_bins - 1);
    int blue_bin =
        std::min(static_cast<int>(floor(blue / bin_size)), num_bins - 1);
    int index = red_bin * num_bins * num_bins + green_bin * num_bins + blue_bin;
    (*histogram)[index] += 1;
  }
}

float SquaredDistance(const PointXYZRGB& p1, const PointXYZRGB& p2) {
  float distance = 0;
  // Color ranges from 0-255 while distances range about 0.15m, so we increase
  // the importance of distance.
  float distance_scale_factor = 1.0 / (0.15 * 0.15);
  float color_scale_factor = 1.0 / (255 * 255);
  distance += distance_scale_factor * (p1.x - p2.x) * (p1.x - p2.x);
  distance += distance_scale_factor * (p1.y - p2.y) * (p1.y - p2.y);
  distance += distance_scale_factor * (p1.z - p2.z) * (p1.z - p2.z);
  distance += color_scale_factor * (p1.r - p2.r) * (p1.r - p2.r);
  distance += color_scale_factor * (p1.g - p2.g) * (p1.g - p2.g);
  distance += color_scale_factor * (p1.b - p2.b) * (p1.b - p2.b);
  return distance;
}

bool ClusterWithKMeans(const PointCloud<PointXYZRGB>& cloud,
                       const int num_clusters,
                       vector<PointCloud<PointXYZRGB>::Ptr>* clusters) {
  ROS_INFO("KMeans point cloud size: %ld", cloud.size());
  vector<PointXYZRGB> centroids;
  vector<vector<PointXYZRGB> > clusters_vector;

  // Initialize centroids with spatial variance in the y direction, but randomly
  // otherwise.
  PointXYZRGB cloud_min;
  PointXYZRGB cloud_max;
  pcl::getMinMax3D(cloud, cloud_min, cloud_max);
  ROS_INFO("Min: %f %f %f %d %d %d", cloud_min.x, cloud_min.y, cloud_min.z,
           cloud_min.r, cloud_min.g, cloud_min.b);
  ROS_INFO("Max: %f %f %f %d %d %d", cloud_max.x, cloud_max.y, cloud_max.z,
           cloud_max.r, cloud_max.g, cloud_max.b);
  float y_increment = (cloud_max.y - cloud_min.y) / (num_clusters - 1);
  for (int i = 0; i < num_clusters; ++i) {
    PointXYZRGB centroid;
    centroid.x = (cloud_max.x - cloud_min.x) / 2.0 + cloud_min.x;
    centroid.y = cloud_min.y + y_increment * i;
    centroid.z = (cloud_max.z - cloud_min.z) / 2.0 + cloud_min.z;
    centroid.r = rand() % 256;
    centroid.g = rand() % 256;
    centroid.b = rand() % 256;
    centroids.push_back(centroid);
    ROS_INFO("Initial centroid %d: %f %f %f %d %d %d", i, centroid.x,
             centroid.y, centroid.z, centroid.r, centroid.g, centroid.b);
  }

  // Iterate
  for (int i = 0; i < 100; ++i) {
    // Compute cluster assignments.
    clusters_vector.clear();
    for (int c = 0; c < num_clusters; ++c) {
      vector<PointXYZRGB> cloud;
      clusters_vector.push_back(cloud);
    }

    for (size_t p = 0; p < cloud.size(); ++p) {
      const PointXYZRGB& point = cloud[p];
      int best_cluster = 0;
      float best_distance = SquaredDistance(point, centroids[0]);
      for (size_t c = 1; c < centroids.size(); ++c) {
        float distance = SquaredDistance(point, centroids[c]);
        if (distance < best_distance) {
          best_cluster = c;
        }
      }
      clusters_vector[best_cluster].push_back(point);
    }

    for (size_t c = 0; c < clusters_vector.size(); ++c) {
      const vector<PointXYZRGB>& cluster = clusters_vector[c];
      if (cluster.size() == 0) {
        ROS_ERROR(
            "[ItemSegmentation] Cluster %ld is of size 0 in K-means algorithm.",
            c);
        return false;
      }
    }

    bool converged = true;

    // Compute centroids.
    for (size_t c = 0; c < clusters_vector.size(); ++c) {
      const vector<PointXYZRGB>& cluster = clusters_vector[c];
      PointXYZRGB centroid;
      centroid.x = 0;
      centroid.y = 0;
      centroid.z = 0;
      float centroid_r = 0;
      float centroid_g = 0;
      float centroid_b = 0;
      for (size_t p = 0; p < cluster.size(); ++p) {
        const PointXYZRGB& point = cluster[p];
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
        centroid_r += point.r;
        centroid_g += point.g;
        centroid_b += point.b;
      }
      centroid.x /= cluster.size();
      centroid.y /= cluster.size();
      centroid.z /= cluster.size();
      centroid_r /= cluster.size();
      centroid_g /= cluster.size();
      centroid_b /= cluster.size();
      centroid.r = static_cast<uint8_t>(centroid_r);
      centroid.g = static_cast<uint8_t>(centroid_g);
      centroid.b = static_cast<uint8_t>(centroid_b);

      if (SquaredDistance(centroids[c], centroid) > 0.001) {
        converged = false;
      }
      centroids[c] = centroid;
      ROS_INFO("%d, %ld: (%f, %f, %f, %d, %d, %d)", i, c, centroid.x,
               centroid.y, centroid.z, centroid.r, centroid.g, centroid.b);
    }

    if (converged) {
      break;
    }
  }
  for (size_t c = 0; c < clusters_vector.size(); ++c) {
    ROS_INFO("K-means cluster %ld size=%ld", c, clusters_vector[c].size());
  }

  clusters->clear();
  for (size_t i = 0; i < clusters_vector.size(); ++i) {
    const vector<PointXYZRGB>& cloud_vec = clusters_vector[i];
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>());
    cloud->width = cloud_vec.size();
    cloud->height = 1;
    for (size_t j = 0; j < cloud_vec.size(); ++j) {
      cloud->push_back(cloud_vec[j]);
    }
    clusters->push_back(cloud);
  }
  return true;
}

void ClusterWithRegionGrowing(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* clusters) {
  pcl::RegionGrowingRGB<PointXYZRGB> reg;
  reg.setInputCloud(cloud.makeShared());

  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>());
  tree->setInputCloud(cloud.makeShared());
  reg.setSearchMethod(tree);

  float distance_threshold;
  ros::param::param<float>("distance_threshold", distance_threshold, 0.05);
  reg.setDistanceThreshold(distance_threshold);

  float point_color;
  ros::param::param<float>("point_color", point_color, 10);
  float region_color;
  ros::param::param<float>("region_color", region_color, 20);
  reg.setPointColorThreshold(point_color);
  reg.setRegionColorThreshold(region_color);

  int min_cluster_size;
  ros::param::param<int>("min_cluster_size", min_cluster_size, 150);
  reg.setMinClusterSize(min_cluster_size);

  std::vector<pcl::PointIndices> clusters_ind;
  reg.extract(clusters_ind);

  clusters->clear();
  clusters->resize(clusters_ind.size());
  for (size_t i = 0; i < clusters_ind.size(); ++i) {
    const pcl::PointIndices& cluster_ind = clusters_ind[i];
    PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>());
    // ROS_INFO("[Region growing] Cluster %ld has size %ld", i,
    //         cluster_ind.indices.size());
    for (size_t j = 0; j < cluster_ind.indices.size(); ++j) {
      const int index = cluster_ind.indices[j];
      cluster->push_back(cloud[index]);
    }
    (*clusters)[i] = cluster;
  }
}

namespace {
// A PointCloud coupled with its min and max y points.
struct PointCloudExtent {
  PointCloudExtent(PointCloud<PointXYZRGB>::Ptr pc, float min_y, float max_y)
      : pc_(pc), min_y_(min_y), max_y_(max_y) {}
  PointCloud<PointXYZRGB>::Ptr pc_;
  float min_y_;
  float max_y_;
};

// Compares two PointCloudExtents by Y value.
bool ComparePointCloudExtentsByY(const PointCloudExtent& a,
                                 const PointCloudExtent& b) {
  return a.min_y_ < b.min_y_;
}

float OverlapPercentage(const PointCloudExtent& a, const PointCloudExtent& b) {
  float a_width = a.max_y_ - a.min_y_;
  float b_width = b.max_y_ - b.min_y_;
  float min_width = b_width;
  if (a_width < b_width) {
    min_width = a_width;
  }

  // Returns the overlap as a percentage of the min_width.
  float overlap = 0;
  if (a.min_y_ < b.min_y_) {
    if (b.min_y_ < a.max_y_ && b.max_y_ < a.max_y_) {
      overlap = b.max_y_ - b.min_y_;
    } else if (b.min_y_ < a.max_y_ && b.max_y_ >= a.max_y_) {
      overlap = a.max_y_ - b.min_y_;
    } else {
      overlap = 0;
    }
  } else {
    if (a.min_y_ < b.max_y_ && a.max_y_ < b.max_y_) {
      overlap = a.max_y_ - a.min_y_;
    } else if (a.min_y_ < b.max_y_ && a.max_y_ >= b.max_y_) {
      overlap = b.max_y_ - a.min_y_;
    } else {
      overlap = 0;
    }
  }

  return overlap / min_width;
}
}

void ClusterBinItems(const PointCloud<PointXYZRGB>& cloud,
                     const int num_clusters,
                     std::vector<PointCloud<PointXYZRGB>::Ptr>* clusters) {
  std::vector<PointCloud<PointXYZRGB>::Ptr> overclustering;
  ClusterWithRegionGrowing(cloud, &overclustering);

  vector<PointCloudExtent> sorted_overclustering;
  for (size_t i = 0; i < overclustering.size(); ++i) {
    const PointCloud<PointXYZRGB>::Ptr& overcluster = overclustering[i];
    pcl::PointXYZRGB min;
    pcl::PointXYZRGB max;
    pcl::getMinMax3D(*overcluster, min, max);
    sorted_overclustering.push_back(
        PointCloudExtent(overcluster, min.y, max.y));
  }
  std::sort(sorted_overclustering.begin(), sorted_overclustering.end(),
            ComparePointCloudExtentsByY);

  vector<PointCloudExtent> output_clusters;
  for (double overlap_threshold = 0.5; overlap_threshold >= 0.01;
       overlap_threshold -= 0.1) {
    output_clusters.clear();
    for (size_t i = 0; i < sorted_overclustering.size(); ++i) {
      PointCloudExtent& subcluster = sorted_overclustering[i];
      if (output_clusters.size() == 0) {
        PointCloud<PointXYZRGB>::Ptr extent_pc(new PointCloud<PointXYZRGB>());
        (*extent_pc) = (*subcluster.pc_);
        PointCloudExtent extent(extent_pc, subcluster.min_y_,
                                subcluster.max_y_);
        output_clusters.push_back(extent);
        continue;
      }

      // Compare to all subclusters, see if they overlap.
      bool merged_subcluster = false;
      for (size_t j = 0; j < output_clusters.size(); ++j) {
        PointCloudExtent& output_cluster = output_clusters[j];
        float overlap_percentage =
            OverlapPercentage(output_cluster, subcluster);
        // ROS_INFO(
        //    "Subcluster (%f, %f) overlaps with output cluster %ld (%f, %f) "
        //    "with score %f",
        //    subcluster.min_y_, subcluster.max_y_, j, output_cluster.min_y_,
        //    output_cluster.max_y_, overlap_percentage);
        if (overlap_percentage >= overlap_threshold) {
          // Merge subcluster into output cluster.
          (*output_cluster.pc_) += (*subcluster.pc_);
          pcl::PointXYZRGB min;
          pcl::PointXYZRGB max;
          pcl::getMinMax3D(*output_cluster.pc_, min, max);
          output_cluster.min_y_ = min.y;
          output_cluster.max_y_ = max.y;
          merged_subcluster = true;
          break;
        }
      }
      if (!merged_subcluster) {
        PointCloud<PointXYZRGB>::Ptr extent_pc(new PointCloud<PointXYZRGB>());
        (*extent_pc) = (*subcluster.pc_);
        PointCloudExtent extent(extent_pc, subcluster.min_y_,
                                subcluster.max_y_);

        output_clusters.push_back(extent);
      }
    }
    if (static_cast<int>(output_clusters.size()) < num_clusters) {
      ROS_WARN("Got %ld clusters with threshold %f, less than expected (%d)",
               output_clusters.size(), overlap_threshold, num_clusters);
      break;
    } else if (static_cast<int>(output_clusters.size()) == num_clusters) {
      ROS_INFO("Got %d clusters with overlap threshold of %f", num_clusters,
               overlap_threshold);
      break;
    } else {
      ROS_INFO("Got %ld clusters with threshold %f, more than expected (%d)",
               output_clusters.size(), overlap_threshold, num_clusters);
    }
  }

  clusters->clear();
  for (size_t i = 0; i < output_clusters.size(); ++i) {
    PointCloudExtent& output_cluster = output_clusters[i];
    PointCloud<PointXYZRGB>::Ptr cluster = output_cluster.pc_;
    cluster->height = 1;
    cluster->width = cluster->size();
    clusters->push_back(cluster);
  }
}

void ClusterWithEuclidean(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
    const double cluster_tolerance, const int min_cluster_size,
    const int max_cluster_size,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* clusters) {
  pcl::search::KdTree<PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGB>);
  tree->setInputCloud(cloud.makeShared());
  std::vector<pcl::PointIndices> clustersInd;
  pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
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

void ComputeCentroidXYZRGB(const PointCloud<PointXYZRGB>& cluster,
                           PointXYZRGB* centroid) {
  centroid->x = 0;
  centroid->y = 0;
  centroid->z = 0;
  float centroid_r = 0;
  float centroid_g = 0;
  float centroid_b = 0;
  for (size_t p = 0; p < cluster.size(); ++p) {
    const PointXYZRGB& point = cluster[p];
    centroid->x += point.x;
    centroid->y += point.y;
    centroid->z += point.z;
    centroid_r += point.r;
    centroid_g += point.g;
    centroid_b += point.b;
  }
  centroid->x /= cluster.size();
  centroid->y /= cluster.size();
  centroid->z /= cluster.size();
  centroid_r /= cluster.size();
  centroid_g /= cluster.size();
  centroid_b /= cluster.size();
  centroid->r = static_cast<uint8_t>(centroid_r);
  centroid->g = static_cast<uint8_t>(centroid_g);
  centroid->b = static_cast<uint8_t>(centroid_b);
}

void ClusterBinItemsWithKMeans(
    const PointCloud<PointXYZRGB>& cloud, const int num_clusters,
    std::vector<PointCloud<PointXYZRGB>::Ptr>* clusters) {
  std::vector<PointCloud<PointXYZRGB>::Ptr> overclustering;
  ClusterWithRegionGrowing(cloud, &overclustering);

  PointCloud<PointXYZRGB> centroids;
  for (size_t i = 0; i < overclustering.size(); ++i) {
    const PointCloud<PointXYZRGB>::Ptr& overcluster = overclustering[i];
    PointXYZRGB centroid;
    ComputeCentroidXYZRGB(*overcluster, &centroid);
    centroids.push_back(centroid);
  }

  std::vector<PointCloud<PointXYZRGB>::Ptr> centroid_clusters;

  // Sometimes K-means has no points in a cluster, try again if so.
  for (int tries = 0; tries < 10; ++tries) {
    centroid_clusters.clear();
    bool success =
        ClusterWithKMeans(centroids, num_clusters, &centroid_clusters);
    if (success) {
      break;
    }
  }

  // Merge clusters together.
  clusters->clear();
  clusters->resize(num_clusters);
  for (size_t i = 0; i < centroid_clusters.size(); ++i) {
    PointCloud<PointXYZRGB>::Ptr centroids_in_cluster = centroid_clusters[i];

    PointCloud<PointXYZRGB>::Ptr cluster(new PointCloud<PointXYZRGB>());
    cluster->height = 1;

    for (size_t j = 0; j < centroids_in_cluster->size(); ++j) {
      const PointXYZRGB& centroid = (*centroids_in_cluster)[j];
      int centroid_index = 0;
      for (size_t k = 0; k < centroids.size(); ++k) {
        if (pcl::squaredEuclideanDistance(centroid, centroids.points[k]) == 0) {
          centroid_index = k;
          break;
        }
      }
      const PointCloud<PointXYZRGB>::Ptr& prev_cluster =
          overclustering[centroid_index];
      (*cluster) += (*prev_cluster);
      // for (size_t k = 0; k < prev_cluster->size(); ++k) {
      //  cluster->push_back((*prev_cluster)[k]);
      //}
      // cluster->width += prev_cluster->size();
    }
    (*clusters)[i] = cluster;
  }
}
}  // namespace pr2_pick_perception
