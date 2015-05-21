# Contains a priori information about an item.
bool is_graspable
string[] allowed_grasps
bool allow_finger_collisions
bool grasp_wide_end
bool grasp_multiple_heights
bool zero_width_grasp # Whether the object could be picked up with a completely closed gripper.
int32 bonus_points
float64 grasp_effort # -1 for maximum strength, ~10 for less.
float64 success_prior # Number between 0 - 1, likelihood of success for this item.
