# Message describing an item's physical attributes

# Transform from some known frame to the item frame.
geometry_msgs/TransformStamped transform

# All the cluster's points lie within the bounding box. It is axis-aligned
# with the item frame, and located in the first octant, with corners at
# (0, 0, 0) and (bbox.x, bbox.y, bbox.z) in the item frame.
geometry_msgs/Point bbox

# A color histogram.
pr2_pick_perception/ColorHistogram histogram

# Planar bounding box.
# The x direction points in the direction of the principal component.
# The z direction points up.
pr2_pick_perception/BoundingBox planar_bounding_box
