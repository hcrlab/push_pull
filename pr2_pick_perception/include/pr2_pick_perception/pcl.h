#ifndef _PR2_PICK_PERCEPTION_PCL_H_
#define _PR2_PICK_PERCEPTION_PCL_H_

#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/filters/filter.h"

namespace pr2_pick_perception {
// Computes the principal components of the given point cloud on the XY plane.
// The XY plane is defined by the frame_id given in the cloud.
//
// Args:
//   cloud: The input point cloud to get the principal components of.
//   component1: The orientation of the principal component in the input cloud's
//     frame.
//   component2: The orientation of the smaller component.
//   value1: The eigenvalue of the principal component.
//   value2: The eigenvalue of the smaller component.
void PlanarPrincipalComponents(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                               geometry_msgs::Quaternion* component1,
                               geometry_msgs::Quaternion* component2,
                               double* value1, double* value2);

/**
 * Given a cluster, construct a bounding box determined by the cluster's planar
 * principal components, with a reference frame at one corner of the bounding
 * box.
 *
 * Preconditions: cloud does not contain any NaN points
 *
 * @param cloud (in) cluster of points representing an item
 * @param transform (out) transform from cloud's frame to item's frame
 * @param bbox (out) corner of the item's axis-aligned bounding box opposite
 *                   the item's origin
 */
void GetBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                    geometry_msgs::TransformStamped* transform,
                    geometry_msgs::Point* bbox);

// Computes the bounding box for the given point cloud on the XY plane.
//
// Args:
//   cloud: The input cloud. Assumed to not have any NaN points.
//   midpoint: The geometric center of the cloud, i.e., the midpoint between the
//     minimum and maximum points in each of the x, y, and z directions. The
//     orientation is such that the x direction points along the principal
//     component in the XY plane, the y direction points along the smaller
//     component, and the z direction points up.
//   dimensions: A vector containing the length of the cloud in the x, y, and z
//   directions.
void GetPlanarBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                          geometry_msgs::Pose* midpoint,
                          geometry_msgs::Vector3* dimensions);

// Computes the color histogram of the given point cloud with RGB information.
//
// This version produces separate histograms for the R, G, and B values.
//
// Operates on point clouds with NaNs filtered out
// (pcl::removeNanFromPointCloud).
//
// Args:
//   cloud: The input point cloud.
//   num_bins: The number of bins for each color channel in the histogram.
//   histogram: The output histogram. The vector is laid out with num_bins
//     values for the red channel, num_bins values for the blue channel, and
//     num_bins values for the green channel.
void ComputeColorHistogramSeparate(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const int num_bins,
    std::vector<int>* histogram);

// Computes the color histogram of the given point cloud with RGB information.
//
// This version produces a joint histogram with R, G, and B values. For example,
// if there are 2 bins, then there are 2^3 values in the histogram. The
// histogram is
// in order of [count(r0, g0, b0), count(r0, g0, b1), count(r0, g1, b0), ...,
// count(r1, g1, b1)], where r0 signifies a value between 0-127 and r1 signifies
// a value of 128 - 255.
//
// Operates on point clouds with NaNs filtered out
// (pcl::removeNanFromPointCloud).
//
// Args:
//   cloud: The input point cloud.
//   num_bins: The number of bins for each color channel in the histogram.
//   histogram: The output histogram. The vector is laid out with num_bins
//     values for the red channel, num_bins values for the blue channel, and
//     num_bins values for the green channel.
void ComputeColorHistogram(const pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                           const int num_bins, std::vector<int>* histogram);

// Returns the squared Euclidean distance between the two points.
float SquaredDistance(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2);

// Clusters the point cloud with K-means, using both distance and color
// information.
//
// Args:
//   cloud: The input point cloud.
//   num_clusters: The number of clusters to extract.
//   clusters: The returned clusters.
void ClusterWithKMeans(
    const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const int num_clusters,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> >* clusters);
}  // namespace pr2_pick_perception

#endif
