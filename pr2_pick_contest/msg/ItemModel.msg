# Contains a priori information about an item.
bool is_graspable
string[] allowed_grasps
bool allow_finger_collisions
bool zero_width_grasp # Whether the object could be picked up with a completely closed gripper.
int32 bonus_points
float64 grasp_effort # -1 for maximum strength, ~10 for less.
pr2_pick_perception/ColorHistogram[] color_histograms # A set of pre-recorded color histograms for this object.
