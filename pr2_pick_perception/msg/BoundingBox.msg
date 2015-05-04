# A bounding box for an object.

# The midpoint of the bounding box.
# The x direction points along the principal component of the object.
# Depending on how the bounding box is used, the z direction could point up (planar bounding box), or in the direction of the smallest principal component (oriented bounding box).
geometry_msgs/PoseStamped pose 

# The lengths of the bounding box in each of the x, y, and z directions.
geometry_msgs/Vector3 dimensions
