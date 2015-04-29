# Message describing an item's physical attributes

# Transform from some known frame to the item frame.
geometry_msgs/TransformStamped transform

# All the cluster's points lie within the bounding box. It is axis-aligned
# with the item frame, and located in the first octant, with corners at
# (0, 0, 0) and (bbox.x, bbox.y, bbox.z) in the item frame.
geometry_msgs/Point bbox