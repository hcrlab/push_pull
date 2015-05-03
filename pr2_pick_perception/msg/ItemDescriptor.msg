# Message describing an item's physical attributes

# Transform from some known frame to the item frame.
geometry_msgs/TransformStamped transform

# All the cluster's points lie within the bounding box. It is axis-aligned
# with the item frame, and located in the first octant, with corners at
# (0, 0, 0) and (bbox.x, bbox.y, bbox.z) in the item frame.
geometry_msgs/Point bbox

# A color histogram.
pr2_pick_perception/ColorHistogram histogram

# Minimum bounding box.
# min_bbox_centroid is the pose of the centroid of the box. The orientation is
# such that the x-axis of the box points along the principal component.
# min_bbox_dimensions is a vector such that min_bbox_dimensions.x is the length
# of the box in the x-direction.
geometry_msgs/PoseStamped min_bbox_centroid
geometry_msgs/Vector3 min_bbox_dimensions
