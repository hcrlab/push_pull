#include "pr2_pick_perception/pcl.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/common/centroid.h"
#include "pcl/common/common.h"
#include "pcl/common/pca.h"
#include "pcl/filters/filter.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Dense>
#include <tf/transform_listener.h>

#include <algorithm>
#include <math.h>
#include <vector>

using geometry_msgs::PoseStamped;
using geometry_msgs::Quaternion;
using geometry_msgs::Vector3;
using pcl::PointCloud;
using pcl::PointXYZRGB;

namespace pr2_pick_perception {
void PlanarPrincipalComponents(const PointCloud<PointXYZRGB>& cloud,
                               geometry_msgs::Quaternion* component1,
                               geometry_msgs::Quaternion* component2,
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

}  // namespace pr2_pick_perception
