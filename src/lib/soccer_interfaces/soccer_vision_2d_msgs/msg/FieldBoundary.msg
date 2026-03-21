# Describes a detected field boundary from vision modules, in image coordinates.
# The field boundary is defined as the line that separates the field of play, to
# outside the field of play. Field of play is a sports terminology.
# Usually, this is where the field carpet ends, or meets a field border.
# If no field boundary was detected, leave the "points" vector empty.

std_msgs/Header header

# The points along the field boundary detected in the image, are stored as a line strip.
# You can think of the field boundary as a line between every two consecutive points,
# so 0-1, 1-2, 2-3, 3-4, 4-5...
vision_msgs/Point2D[] points # 2D points (in pixel)

soccer_vision_attribute_msgs/Confidence confidence
