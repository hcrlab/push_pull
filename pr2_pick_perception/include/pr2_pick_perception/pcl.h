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

}  // namespace pr2_pick_perception

#endif