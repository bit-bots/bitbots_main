# Describes a detected obstacle from vision modules, in 3d coordinates.
# An Obstacle is defined as anything observed on the field that could not be classified as
# one of the known types (such as Robot or Goalpost).

# A bounding box that surrounds the obstacle, this does not have to include
# the whole obstacle, but only the part of the obstacle that was detected.
vision_msgs/BoundingBox3D bb

soccer_vision_attribute_msgs/Confidence confidence
