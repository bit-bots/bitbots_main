# Describes a detected goalpost from vision modules, in image coordinates.

# A bounding box that surrounds the goalpost in the image, that can be rotated about its center.
# This does not have to include the whole goalpost, but only the
# part of the goalpost that was detected.
# The rotation of the bounding box should be 0 when the goalpost appears upright in the image.
vision_msgs/BoundingBox2D bb

soccer_vision_attribute_msgs/Goalpost attributes

soccer_vision_attribute_msgs/Confidence confidence
