# One detected LEGO brick.

# Stable identity. Ground-truth backend: Gazebo model name (e.g. lego_2x4_red_0).
# Image-based detector: tracker id.
string id

# Brick type as studs, e.g. "2x4", "2x2", "1x4".
string brick_type

# Colour name, e.g. "red".
string color

# Pose of the brick's bottom-face centre, in the frame of the enclosing array's header.
# x along the brick's long side, z up out of the studs.
geometry_msgs/Pose pose

# 1.0 for ground truth.
float32 confidence
