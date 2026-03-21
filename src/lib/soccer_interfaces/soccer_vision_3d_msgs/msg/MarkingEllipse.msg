# Describes a detected ellipse marking from vision modules, in 3d coordinates.

# The diameter of the ellipse marking in meters
float64 diameter

# Center pose of the ellipse, where a zero orientation is defined
# as looking directly down at an ellipse marking on the ground.
geometry_msgs/Pose center

soccer_vision_attribute_msgs/Confidence confidence
