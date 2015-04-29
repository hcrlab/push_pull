#ifndef _PR2_PICK_PERCEPTION_PCL_H_
#define _PR2_PICK_PERCEPTION_PCL_H_

#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/PointCloud2.h"

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
void PlanarPrincipalComponents(const sensor_msgs::PointCloud2& cloud,
                               geometry_msgs::Quaternion* component1,
                               geometry_msgs::Quaternion* component2,
                               double* value1, double* value2);

/**
 * Given a cluster, construct a bounding box determined by the cluster's planar
 * principal components, with a reference frame at one corner of the bounding box.
 * @param cloud (in) cluster of points representing an item
 * @param transform (out) transform from cloud's frame to item's frame
 * @param bbox (out) corner of the item's axis-aligned bounding box opposite
 *                   the item's origin
 */
void BoundingBox(const sensor_msgs::PointCloud2& cloud,
                 geometry_msgs::TransformStamped* transform,
                 geometry_msgs::Point* bbox);

}  // namespace pr2_pick_perception

#endif
