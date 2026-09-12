nav_msgs/Path path
# Actual fourth-joint angles in radians, one per path pose, without wrapping.
float64[] wrist_angles
# Optional unwrapped field-yaw interpolation; empty uses wrist interpolation.
float64[] phi_angles
# Path sample index corresponding to each route target.
uint32[] target_sample_indices
