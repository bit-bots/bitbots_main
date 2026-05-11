# Describes a detected ball from vision modules, in image coordinates.

# A bounding box that surrounds the ball in the image, this does not have to include
# the whole ball, but only the part of the ball that was detected.
vision_msgs/BoundingBox2D bb

# Center point of the ball in pixel coordinates
vision_msgs/Point2D center

# Confidence that the detected object is a ball
soccer_vision_attribute_msgs/Confidence confidence
